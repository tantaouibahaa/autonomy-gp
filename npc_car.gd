extends Node3D

## NPC traffic car. Drives along the TrafficNetwork lane graph, obeys traffic
## lights, yields at roundabouts, keeps its distance to anything in its path
## (cars, pedestrians, obstacles, the player) and changes lane around
## parked cars or road works. Movement is kinematic: the car is placed on its
## lane every physics frame and its AnimatableBody3D pushes physics bodies.

## Car models to pick from at random. Kenney models face +Z.
@export var models: Array[PackedScene] = []
@export var model_scale := 1.5
@export var accel := 2.5
@export var comfort_decel := 3.5
@export var max_decel := 8.0
## Lateral acceleration allowed in curves, m/s^2.
@export var curve_accel := 4.0
## Chance to take an off-ramp when passing one in the right lane.
@export var exit_chance := 0.35
## Bumper to bumper gap kept to whatever is ahead, in meters.
@export var min_gap := 3.0
## Fraction of the speed limit this driver goes, picked at random.
@export var speed_factor_range := Vector2(0.85, 1.0)

const PROBE_MASK := 2 | 4 | 8 # vehicles, pedestrians, obstacles
const PROBE_RADIUS := 1.1
const PROBE_STEPS := [3.0, 5.5, 8.5, 12.0, 16.0, 21.0, 27.0, 34.0]
const LANE_CHANGE_TIME := 1.6

@onready var agent: RoadLaneAgent = %road_lane_agent
@onready var body: AnimatableBody3D = $Body

var network: TrafficNetwork
var lane: RoadLane
var offset := 0.0
var speed := 0.0
## What stopped the car this frame: "", "light", "yield", "car", "pedestrian", "obstacle".
var blocked_by := ""

var _next := {} # chosen successor of the current lane
var _speed_factor := 1.0
var _half_length := 2.2
var _blocked_time := 0.0
var _lateral := Vector3.ZERO # leftover sideways offset after a lane change
var _query := PhysicsShapeQueryParameters3D.new()


func _ready() -> void:
	if not models.is_empty():
		var model: Node3D = models.pick_random().instantiate()
		model.scale = Vector3.ONE * model_scale
		model.rotation.y = PI # cars drive along -Z
		add_child(model)
		_fit_body(model)
	_speed_factor = randf_range(speed_factor_range.x, speed_factor_range.y)
	var sphere := SphereShape3D.new()
	sphere.radius = PROBE_RADIUS
	_query.shape = sphere
	_query.collision_mask = PROBE_MASK
	_query.exclude = [body.get_rid()]


## Put the car on `new_lane` at `new_offset` (meters from the lane start).
func place(new_network: TrafficNetwork, new_lane: RoadLane, new_offset: float) -> void:
	network = new_network
	_enter_lane(new_lane, new_offset)
	speed = network.speed_limit(lane) * 0.5
	_update_transform(0.0)


func _physics_process(delta: float) -> void:
	if lane == null or not is_instance_valid(lane):
		return
	var target := _target_speed()
	if target > speed:
		speed = minf(target, speed + accel * delta)
	else:
		speed = maxf(target, speed - max_decel * delta)

	if _blocked_time > 1.5:
		_try_lane_change()

	var old_offset := offset
	offset += speed * delta
	for d in network.diverges(lane):
		if d.at > old_offset and d.at <= offset and randf() < exit_chance:
			_change_to(d.lane, offset - d.at)
			break
	while lane and offset > network.lane_length(lane):
		var overflow := offset - network.lane_length(lane)
		if _next.is_empty():
			offset = network.lane_length(lane)
			speed = 0.0
			break
		_enter_lane(_next.lane, _next.offset + overflow)
	_update_transform(delta)


func _target_speed() -> float:
	blocked_by = ""
	var limit := network.speed_limit(lane) * _speed_factor
	var target := limit

	# Slow down for curves ahead.
	var here: Array = _path_point(0.0)
	for look in [8.0, 16.0]:
		var ahead: Array = _path_point(look)
		var angle := acos(clampf(here[1].dot(ahead[1]), -1.0, 1.0))
		if angle > 0.05:
			target = minf(target, sqrt(curve_accel * look / angle))

	# Traffic light.
	if lane.has_meta("signal"):
		var controller: TrafficLightController = lane.get_meta("signal")
		var stop_at: float = lane.get_meta("stop_offset")
		var dist := stop_at - offset
		if dist > -0.5:
			var light: int = controller.get_state(lane.get_meta("arm"))
			var can_stop := dist > speed * speed / (2.0 * comfort_decel) - 1.0
			if light == TrafficLightController.Light.RED \
					or (light == TrafficLightController.Light.YELLOW and can_stop):
				target = minf(target, _stop_speed(dist - 0.5))
				if target < 0.5:
					blocked_by = "light"

	# Roundabout entry: wait while cars circulate near our entry point.
	if lane.has_meta("yield_at"):
		var dist := network.lane_length(lane) - offset
		if dist < 14.0 and _occupied(_path_point(dist + 6.0)[0], 6.0, 2):
			target = minf(target, _stop_speed(dist - 2.0))
			if target < 0.5:
				blocked_by = "yield"

	# Anything in our path.
	var reach := maxf(12.0, speed * speed / (2.0 * comfort_decel) + min_gap + 10.0)
	for step in PROBE_STEPS:
		if step > reach:
			break
		_query.transform = Transform3D(Basis.IDENTITY, _path_point(step)[0] + Vector3.UP * 0.9)
		var hits := get_world_3d().direct_space_state.intersect_shape(_query, 4)
		if hits.is_empty():
			continue
		var gap: float = step - PROBE_RADIUS - _half_length - min_gap
		target = minf(target, _stop_speed(gap))
		var collider: Object = hits[0].collider
		if collider.is_in_group("obstacles"):
			blocked_by = "obstacle"
		elif collider.is_in_group("pedestrians"):
			blocked_by = "pedestrian"
		else:
			blocked_by = "car"
		break

	if blocked_by == "obstacle" and speed < 1.0:
		_blocked_time += get_physics_process_delta_time()
	else:
		_blocked_time = 0.0
	return maxf(target, 0.0)


## Highest speed from which the car can still stop within `dist` meters.
## Zero inside the last few centimeters, so the car stops instead of creeping on.
func _stop_speed(dist: float) -> float:
	return sqrt(2.0 * comfort_decel * dist) if dist > 0.05 else 0.0


func _occupied(pos: Vector3, radius: float, mask: int) -> bool:
	var q := PhysicsShapeQueryParameters3D.new()
	var s := SphereShape3D.new()
	s.radius = radius
	q.shape = s
	q.collision_mask = mask
	q.exclude = [body.get_rid()]
	q.transform = Transform3D(Basis.IDENTITY, pos + Vector3.UP)
	return not get_world_3d().direct_space_state.intersect_shape(q, 1).is_empty()


## Point and direction `dist` meters ahead along the route.
func _path_point(dist: float) -> Array:
	var l := lane
	var o := offset + dist
	var next := _next
	for _hop in 4:
		var len := network.lane_length(l)
		if o <= len or next.is_empty():
			break
		o = o - len + next.offset
		l = next.lane
		var options := network.npc_successors(l)
		next = options[0] if not options.is_empty() else {}
	return network.sample(l, o)


func _enter_lane(new_lane: RoadLane, new_offset: float) -> void:
	lane = new_lane
	offset = new_offset
	agent.assign_lane(lane)
	var options := network.npc_successors(lane)
	if options.is_empty():
		options = network.successors(lane)
	_next = _pick(options)


func _pick(options: Array) -> Dictionary:
	if options.is_empty():
		return {}
	var total := 0.0
	for o in options:
		total += 2.0 if o.turn == "straight" else 1.0
	var r := randf() * total
	for o in options:
		r -= 2.0 if o.turn == "straight" else 1.0
		if r <= 0.0:
			return o
	return options[-1]


func _try_lane_change() -> void:
	for n in network.neighbours(lane):
		var off: float = n.lane.curve.get_closest_offset(n.lane.to_local(global_position))
		# Room beside us, and nobody coming up close behind in that lane.
		var free := true
		for back in [0.0, 8.0, 16.0]:
			if _occupied(network.sample(n.lane, off - back)[0], 1.8, 2):
				free = false
		if free:
			_change_to(n.lane, off)
			_blocked_time = 0.0
			return


func _change_to(new_lane: RoadLane, new_offset: float) -> void:
	var before := global_position
	_enter_lane(new_lane, new_offset)
	_lateral = before - network.sample(lane, offset)[0]


func _update_transform(delta: float) -> void:
	if delta > 0.0:
		_lateral = _lateral.move_toward(Vector3.ZERO, delta * maxf(_lateral.length(), 1.0) / LANE_CHANGE_TIME * 1.5)
	var here: Array = _path_point(0.0)
	var ahead: Array = _path_point(2.5)
	var pos: Vector3 = here[0] + _lateral
	var look: Vector3 = ahead[0] + _lateral * 0.5
	global_position = pos
	if not pos.is_equal_approx(look):
		look_at(look, Vector3.UP)


## Size the collision box to the model.
func _fit_body(model: Node3D) -> void:
	var box := AABB()
	var first := true
	for mi in model.find_children("*", "MeshInstance3D", true, false):
		var a: AABB = (global_transform.affine_inverse() * mi.global_transform) * mi.mesh.get_aabb()
		box = a if first else box.merge(a)
		first = false
	if first:
		return
	var shape := BoxShape3D.new()
	shape.size = box.size
	var cs: CollisionShape3D = body.get_node("CollisionShape3D")
	cs.shape = shape
	cs.position = box.get_center()
	_half_length = box.size.z / 2.0
