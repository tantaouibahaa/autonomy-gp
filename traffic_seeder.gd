extends Node

## Places NPC cars on random lanes of the TrafficNetwork when the scene starts.

## Node with road_actor_manager.gd that owns the NPC cars.
@export var vehicles_path: NodePath
## Optional node (e.g. the player car) that no NPC should spawn near.
@export var keep_clear_of: NodePath
@export var car_count := 50
## Minimum distance in meters between seeded cars.
@export var min_spacing := 20.0
## Keep this far from the player car.
@export var clear_radius := 40.0


func _ready() -> void:
	var network := get_tree().get_first_node_in_group("traffic_network") as TrafficNetwork
	if network == null:
		push_warning("No TrafficNetwork in the scene, no traffic seeded")
		return
	if not network.is_built:
		await network.built
	_seed_cars(network)


func _seed_cars(network: TrafficNetwork) -> void:
	var vehicles := get_node(vehicles_path)
	var clear_node := get_node_or_null(keep_clear_of) as Node3D
	# Spawn on ordinary road lanes only, weighted by length, never in junctions.
	var lanes := network.lanes.filter(func(l): return not l.has_meta("turn") \
			and network.lane_length(l) > 30.0 and network._junction_of(l) == null)
	if lanes.is_empty():
		push_warning("TrafficNetwork has no lanes, no traffic seeded")
		return
	var weights := PackedFloat32Array()
	for l in lanes:
		weights.append(network.lane_length(l))
	var rng := RandomNumberGenerator.new()
	rng.randomize()

	var taken: Array[Vector3] = []
	var placed := 0
	for _attempt in car_count * 30:
		if placed >= car_count:
			break
		var lane: RoadLane = lanes[rng.rand_weighted(weights)]
		var len := network.lane_length(lane)
		var offset := rng.randf_range(5.0, len - 15.0)
		var pos: Vector3 = network.sample(lane, offset)[0]
		if clear_node and pos.distance_to(clear_node.global_position) < clear_radius:
			continue
		if taken.any(func(p): return p.distance_to(pos) < min_spacing):
			continue
		if _blocked(pos):
			continue
		taken.append(pos)
		var car = vehicles.add_actor(pos, lane)
		if car:
			car.place(network, lane, offset)
			placed += 1


## True if a parked car or road works sits at `pos`.
func _blocked(pos: Vector3) -> bool:
	var q := PhysicsShapeQueryParameters3D.new()
	var s := SphereShape3D.new()
	s.radius = 4.0
	q.shape = s
	q.collision_mask = 2 | 8
	q.transform = Transform3D(Basis.IDENTITY, pos + Vector3.UP)
	return not get_viewport().world_3d.direct_space_state.intersect_shape(q, 1).is_empty()
