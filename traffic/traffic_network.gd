class_name TrafficNetwork
extends Node3D

## Lane graph over every RoadLane in the scene, built at startup.
##
## The road addon does not link lanes across RoadContainers or through junction
## prefabs, so this node does it: lane ends are joined to lane starts (or merge
## points) found nearby, and each junction in the "junctions" group gets its
## own connector lanes for every straight / left / right movement. The prefab's
## own junction lanes are taken out of the lane group.
##
## Lanes ending at a junction get metadata other code can read:
##   "junction"     the junction RoadContainer
##   "arm"          index of the arm the lane arrives on
##   "signal"       its TrafficLightController (signalized junctions only)
##   "stop_offset"  where a car's centre should stop, as a lane offset
##   "yield_at"     the roundabout to yield at (roundabouts only)
## Connector lanes carry "turn" ("straight", "left", "right") and "npc_ok".

signal built

const LINK_DIST := 1.5
const JUNCTION_GROUP := "junctions"
const ROUNDABOUT_GROUP := "roundabouts"

@export var lane_group := "road_lanes"
@export var default_speed_limit := 13.9
@export var turn_speed := 8.0
@export var straight_junction_speed := 12.0
## How far behind the stop line a car's centre stops.
@export var stop_margin := 2.6

## Every lane cars may drive on, connectors included.
var lanes: Array[RoadLane] = []
var is_built := false

var _succ := {} # RoadLane -> Array of {lane, offset, turn}
var _diverge := {} # RoadLane -> Array of {at, lane}
var _neighbours := {} # RoadLane -> Array of {lane, left}
var _length := {} # RoadLane -> float
var _speed := {} # RoadLane -> float
var _bounds := {} # RoadLane -> AABB


func _enter_tree() -> void:
	add_to_group("traffic_network")


func _ready() -> void:
	# Road containers build their lanes during their own _ready.
	await get_tree().process_frame
	await get_tree().process_frame
	build()


func build() -> void:
	var junctions := get_tree().get_nodes_in_group(JUNCTION_GROUP)
	var plain: Array[RoadLane] = []
	for node in get_tree().get_nodes_in_group(lane_group):
		if not node is RoadLane or node.get_parent() == self:
			continue
		var j := _junction_of(node)
		if j and not j.is_in_group(ROUNDABOUT_GROUP):
			node.remove_from_group(lane_group)
			continue
		plain.append(node)
	for lane in plain:
		_register(lane)

	for lane in plain:
		_link_lane(lane, plain)
	for j in junctions:
		if j.is_in_group(ROUNDABOUT_GROUP):
			_mark_roundabout(j, plain)
		else:
			_build_junction(j, plain)
	for lane in plain:
		_find_neighbours(lane, plain)

	var dead := lanes.filter(func(l): return _succ[l].is_empty())
	if not dead.is_empty():
		push_warning("TrafficNetwork: %d lanes lead nowhere, e.g. %s" % [dead.size(), dead[0].get_path()])
	is_built = true
	built.emit()


## Lanes a car can continue onto from the end of `lane`:
## Array of {lane: RoadLane, offset: float, turn: String}.
func successors(lane: RoadLane) -> Array:
	return _succ.get(lane, [])


## Successors NPC traffic may use (no conflicting left turns at signals).
func npc_successors(lane: RoadLane) -> Array:
	return successors(lane).filter(func(s): return s.lane.get_meta("npc_ok", true))


## Lanes branching off part way along `lane` (e.g. an off-ramp): Array of {at, lane}.
func diverges(lane: RoadLane) -> Array:
	return _diverge.get(lane, [])


## Parallel lanes going the same way: Array of {lane, left: bool}.
func neighbours(lane: RoadLane) -> Array:
	return _neighbours.get(lane, [])


func lane_length(lane: RoadLane) -> float:
	return _length.get(lane, lane.curve.get_baked_length())


## Speed limit in m/s, from the lane or the nearest ancestor with "speed_limit".
func speed_limit(lane: RoadLane) -> float:
	if lane in _speed:
		return _speed[lane]
	var node: Node = lane
	while node:
		if node.has_meta("speed_limit"):
			return node.get_meta("speed_limit")
		node = node.get_parent()
	return default_speed_limit


## Point and unit direction at `offset` along `lane`, in world space.
func sample(lane: RoadLane, offset: float) -> Array:
	var len := lane_length(lane)
	offset = clampf(offset, 0.0, len)
	var a := lane.to_global(lane.curve.sample_baked(offset))
	var b := lane.to_global(lane.curve.sample_baked(minf(offset + 0.5, len)))
	if offset + 0.5 > len:
		b = a + (a - lane.to_global(lane.curve.sample_baked(maxf(offset - 0.5, 0.0))))
	return [a, (b - a).normalized()]


## Nearest lane to `pos` and the offset on it, as {lane, offset, distance}.
func nearest(pos: Vector3, max_distance := 10.0) -> Dictionary:
	var best := {}
	var best_d := max_distance
	for lane in lanes:
		if not _bounds[lane].grow(max_distance).has_point(pos):
			continue
		var off := lane.curve.get_closest_offset(lane.to_local(pos))
		var d := lane.to_global(lane.curve.sample_baked(off)).distance_to(pos)
		if d < best_d:
			best_d = d
			best = {lane = lane, offset = off, distance = d}
	return best


func _register(lane: RoadLane) -> void:
	lanes.append(lane)
	_succ[lane] = []
	_length[lane] = lane.curve.get_baked_length()
	var box := AABB(lane.get_lane_start(), Vector3.ZERO)
	for p in lane.curve.get_baked_points():
		box = box.expand(lane.to_global(p))
	_bounds[lane] = box


func _junction_of(node: Node) -> Node:
	while node:
		if node.is_in_group(JUNCTION_GROUP):
			return node
		node = node.get_parent()
	return null


## Join the end of `lane` to lanes that start or run past there, and record
## lanes that split off from it part way along.
func _link_lane(lane: RoadLane, pool: Array[RoadLane]) -> void:
	var len: float = _length[lane]
	var end: Array = sample(lane, len)
	var start: Array = sample(lane, 0.0)
	for other in pool:
		if other == lane:
			continue
		# lane end -> other (its start, or a merge point along it)
		if _bounds[other].grow(LINK_DIST).has_point(end[0]):
			var off := other.curve.get_closest_offset(other.to_local(end[0]))
			var hit: Array = sample(other, off)
			if hit[0].distance_to(end[0]) < LINK_DIST and off < _length[other] - 0.5 \
					and hit[1].dot(end[1]) > 0.7:
				_succ[lane].append({lane = other, offset = off, turn = "straight"})
		# other starts part way along lane -> diverge
		var other_start := other.get_lane_start()
		if _bounds[lane].grow(LINK_DIST).has_point(other_start):
			var off := lane.curve.get_closest_offset(lane.to_local(other_start))
			var hit: Array = sample(lane, off)
			if hit[0].distance_to(other_start) < LINK_DIST and off > 0.5 and off < len - 0.5 \
					and sample(other, 0.0)[1].dot(hit[1]) > 0.7:
				_diverge.get_or_add(lane, []).append({at = off, lane = other})
	# Explicit side links to a lane that starts beside this one part way along
	# (the off-ramp prefab's exit lane).
	for path in [lane.lane_right, lane.lane_left]:
		var side := lane.get_node_or_null(path) as RoadLane
		if side == null or side not in _length:
			continue
		var side_start: Array = sample(side, 0.0)
		var off := lane.curve.get_closest_offset(lane.to_local(side_start[0]))
		var parallel: bool = side_start[0].distance_to(start[0]) < 5.0 \
				and side.get_lane_end().distance_to(end[0]) < 5.0
		if not parallel and off > 0.5 and off < len - 0.5 and side_start[1].dot(sample(lane, off)[1]) > 0.7:
			_diverge.get_or_add(lane, []).append({at = off, lane = side})


func _find_neighbours(lane: RoadLane, pool: Array[RoadLane]) -> void:
	var mid: Array = sample(lane, _length[lane] / 2.0)
	var found := []
	for other in pool:
		if other == lane or not _bounds[other].grow(6.0).has_point(mid[0]):
			continue
		var off := other.curve.get_closest_offset(other.to_local(mid[0]))
		var hit: Array = sample(other, off)
		var d: float = hit[0].distance_to(mid[0])
		if d > 2.5 and d < 5.5 and hit[1].dot(mid[1]) > 0.9:
			found.append({lane = other, left = mid[1].cross(hit[0] - mid[0]).y > 0})
	_neighbours[lane] = found


## Arms of a junction: Array of {pos (edge point), dir (unit, outwards)}.
func _arms(junction: Node3D) -> Array:
	var out := []
	for path in junction.edge_rp_locals:
		var rp: Node3D = junction.get_node(path)
		var dir := rp.global_position - junction.global_position
		dir.y = 0
		if dir.length() > 5.0:
			out.append({pos = rp.global_position, dir = dir.normalized()})
	return out


## Lanes ending (incoming) or starting (outgoing) at each arm, inner lane first.
func _lanes_at_arms(junction: Node3D, arms: Array, pool: Array[RoadLane]) -> Array:
	var result := []
	for arm in arms:
		var incoming := []
		var outgoing := []
		for lane in pool:
			if _junction_of(lane) == junction:
				continue
			for pair in [[lane.get_lane_end(), incoming], [lane.get_lane_start(), outgoing]]:
				var p: Vector3 = pair[0]
				var rel: Vector3 = p - arm.pos
				rel.y = 0
				if absf(rel.dot(arm.dir)) < 1.0 and rel.length() < 12.0:
					pair[1].append({lane = lane, lateral = rel.length()})
		incoming.sort_custom(func(a, b): return a.lateral < b.lateral)
		outgoing.sort_custom(func(a, b): return a.lateral < b.lateral)
		result.append({incoming = incoming.map(func(e): return e.lane),
				outgoing = outgoing.map(func(e): return e.lane)})
	return result


func _build_junction(junction: Node3D, pool: Array[RoadLane]) -> void:
	var arms := _arms(junction)
	var at_arms := _lanes_at_arms(junction, arms, pool)
	var controller := junction.get_node_or_null("TrafficLights") as TrafficLightController
	for a in arms.size():
		var incoming: Array = at_arms[a].incoming
		for lane in incoming:
			_mark_approach(lane, junction, a, controller)
		for b in arms.size():
			if a == b:
				continue
			var outgoing: Array = at_arms[b].outgoing
			if incoming.is_empty() or outgoing.is_empty():
				continue
			var d_in: Vector3 = -arms[a].dir
			var d_out: Vector3 = arms[b].dir
			var side := d_in.cross(d_out).y
			var turn := ""
			var pairs := []
			if d_in.dot(d_out) > 0.7:
				turn = "straight"
				for i in mini(incoming.size(), outgoing.size()):
					pairs.append([incoming[i], outgoing[i]])
			elif side > 0.7:
				turn = "left"
				pairs.append([incoming[0], outgoing[0]])
			elif side < -0.7:
				turn = "right"
				pairs.append([incoming[-1], outgoing[-1]])
			else:
				continue
			var npc_ok := turn != "left" or controller == null or controller.arm_has_own_phase(a)
			for pair in pairs:
				var c := _make_connector(pair[0], pair[1], turn, "%s_%d%d" % [junction.name, a, b])
				c.set_meta("npc_ok", npc_ok)
				c.set_meta("junction", junction)
				_succ[pair[0]].append({lane = c, offset = 0.0, turn = turn})
				_succ[c] = [{lane = pair[1], offset = 0.0, turn = "straight"}]


func _mark_approach(lane: RoadLane, junction: Node3D, arm: int, controller: TrafficLightController) -> void:
	lane.set_meta("junction", junction)
	lane.set_meta("arm", arm)
	if controller:
		var edge_dist := lane.get_lane_end().distance_to(junction.global_position)
		var back := controller.stop_line_distance - edge_dist + stop_margin
		lane.set_meta("signal", controller)
		lane.set_meta("stop_offset", maxf(_length[lane] - back, 0.0))


func _mark_roundabout(junction: Node3D, pool: Array[RoadLane]) -> void:
	var arms := _arms(junction)
	var at_arms := _lanes_at_arms(junction, arms, pool)
	for a in arms.size():
		for lane in at_arms[a].incoming:
			lane.set_meta("junction", junction)
			lane.set_meta("arm", a)
			lane.set_meta("yield_at", junction)


func _make_connector(from: RoadLane, to: RoadLane, turn: String, base_name: String) -> RoadLane:
	var p0: Array = sample(from, _length[from])
	var p3: Array = sample(to, 0.0)
	var chord: float = p0[0].distance_to(p3[0])
	var k := chord * (0.33 if turn == "straight" else 0.39)
	var curve := Curve3D.new()
	curve.bake_interval = 0.5
	curve.add_point(to_local(p0[0]), Vector3.ZERO, p0[1] * k)
	curve.add_point(to_local(p3[0]), -p3[1] * k, Vector3.ZERO)
	var lane := RoadLane.new()
	lane.name = base_name + "_" + turn
	lane.curve = curve
	lane.auto_free_vehicles = false
	lane.set_meta("turn", turn)
	add_child(lane, true)
	lane.add_to_group(lane_group)
	_register(lane)
	_speed[lane] = straight_junction_speed if turn == "straight" else turn_speed
	return lane
