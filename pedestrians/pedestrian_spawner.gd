extends Node3D

## Sidewalk network plus pedestrian spawner.
##
## Sidewalks come from nodes in the "sidewalk_blocks" group, each with a
## "walk_ring" meta (closed PackedVector3Array loop around the block).
## Crosswalks come from nodes in the "crosswalks" group with "a" and "b" metas
## (the two ends, on block rings) whose parent is a TrafficLightController.
## Mid-block jaywalking links are found automatically between facing blocks.

@export var pedestrian_scene: PackedScene
@export var count := 40
## Share of pedestrians that jaywalk now and then.
@export var jaywalker_ratio := 0.15
## Chance a pedestrian uses a crosswalk when passing one.
@export var cross_chance := 0.5
## Chance a jaywalker crosses when passing a mid-block spot.
@export var jaywalk_chance := 0.35
## Ring-to-ring distance across a street, in meters.
@export var street_width := 24.0
## No jaywalking this close to a junction centre.
@export var jaywalk_junction_clearance := 35.0

var rings: Array[PackedVector3Array] = []
var ring_length := PackedFloat32Array()
## Per ring: Array of {s, kind ("crosswalk"/"jaywalk"), a, b, to_ring, to_s, controller}.
var links: Array = []

var _cum: Array[PackedFloat32Array] = []


func _ready() -> void:
	# Wait for the signal controllers and blocks to be ready.
	await get_tree().process_frame
	for block in get_tree().get_nodes_in_group("sidewalk_blocks"):
		if block.has_meta("walk_ring"):
			_add_ring(block.get_meta("walk_ring"))
	for i in rings.size():
		links.append([])
	_add_crosswalks()
	_add_jaywalks()
	_spawn()


## Point on ring `r` at distance `s` along it.
func ring_point(r: int, s: float) -> Vector3:
	var pts := rings[r]
	var cum := _cum[r]
	s = fposmod(s, ring_length[r])
	var i := cum.bsearch(s, true) - 1
	i = clampi(i, 0, pts.size() - 1)
	var a := pts[i]
	var b := pts[(i + 1) % pts.size()]
	var seg := cum[i + 1] - cum[i] if i + 1 < cum.size() else ring_length[r] - cum[i]
	return a.lerp(b, (s - cum[i]) / maxf(seg, 0.001))


## Closest ring point to `p`: {ring, s, distance}, or {} if none within max_distance.
func closest_on_ring(p: Vector3, max_distance := 1.5) -> Dictionary:
	var best := {}
	var best_d := max_distance
	for r in rings.size():
		var pts := rings[r]
		for i in pts.size():
			var a := pts[i]
			var b := pts[(i + 1) % pts.size()]
			var q := Geometry3D.get_closest_point_to_segment(p, a, b)
			var d := Vector2(q.x - p.x, q.z - p.z).length()
			if d < best_d:
				best_d = d
				best = {ring = r, s = _cum[r][i] + a.distance_to(q), distance = d}
	return best


func _add_ring(pts: PackedVector3Array) -> void:
	var cum := PackedFloat32Array([0.0])
	var total := 0.0
	for i in pts.size():
		total += pts[i].distance_to(pts[(i + 1) % pts.size()])
		if i + 1 < pts.size():
			cum.append(total)
	rings.append(pts)
	_cum.append(cum)
	ring_length.append(total)


func _add_link(kind: String, from: Dictionary, to: Dictionary, controller: Node) -> void:
	links[from.ring].append({
		s = from.s, kind = kind, to_ring = to.ring, to_s = to.s, controller = controller,
		a = ring_point(from.ring, from.s), b = ring_point(to.ring, to.s),
	})


func _add_crosswalks() -> void:
	for cw in get_tree().get_nodes_in_group("crosswalks"):
		var a := closest_on_ring(cw.get_meta("a"))
		var b := closest_on_ring(cw.get_meta("b"))
		if a.is_empty() or b.is_empty():
			continue
		_add_link("crosswalk", a, b, cw.get_parent())
		_add_link("crosswalk", b, a, cw.get_parent())


func _add_jaywalks() -> void:
	var junctions := get_tree().get_nodes_in_group("junctions").map(func(j): return j.global_position)
	for r in rings.size():
		var centre := Vector3.ZERO
		for p in rings[r]:
			centre += p
		centre /= rings[r].size()
		var last_s := -INF
		var s := 0.0
		while s < ring_length[r]:
			var p := ring_point(r, s)
			var t := (ring_point(r, s + 0.5) - p).normalized()
			var n := t.cross(Vector3.UP)
			if (p + n - centre).length() < (p - centre).length():
				n = -n
			var near_junction: bool = junctions.any(func(j): return Vector2(j.x - p.x, j.z - p.z).length() < jaywalk_junction_clearance)
			if not near_junction and s - last_s > 30.0:
				var other := closest_on_ring(p + n * street_width, 1.0)
				if not other.is_empty() and other.ring != r:
					_add_link("jaywalk", {ring = r, s = s}, other, null)
					last_s = s
			s += 2.0


func _spawn() -> void:
	if rings.is_empty() or pedestrian_scene == null:
		push_warning("No sidewalks or pedestrian scene, no pedestrians spawned")
		return
	for i in count:
		var ped = pedestrian_scene.instantiate()
		ped.jaywalker = i < int(count * jaywalker_ratio)
		ped.network = self
		ped.ring = randi() % rings.size()
		ped.s = randf() * ring_length[ped.ring]
		ped.walk_dir = 1 if randf() < 0.5 else -1
		add_child(ped)
