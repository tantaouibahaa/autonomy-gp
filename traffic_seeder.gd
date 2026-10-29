extends Node

## Places NPC cars on random road lanes when the scene starts.

## Node with road_actor_manager.gd that owns the NPC cars.
@export var vehicles_path: NodePath
## Optional node (e.g. the player car) that no NPC should spawn near.
@export var keep_clear_of: NodePath
@export var car_count := 12
## Minimum distance in meters between seeded cars.
@export var min_spacing := 15.0
@export var lane_group := "road_lanes"


func _ready() -> void:
	# Road containers generate their lanes on ready; give them a frame.
	await get_tree().process_frame
	_seed_cars()


func _seed_cars() -> void:
	var vehicles := get_node(vehicles_path)
	var clear_node := get_node_or_null(keep_clear_of) as Node3D
	var lanes := get_tree().get_nodes_in_group(lane_group).filter(
		func(l): return l is RoadLane and not l.transition and l.curve.get_baked_length() > min_spacing)
	if lanes.is_empty():
		push_warning("No road lanes found in group '%s', no traffic seeded" % lane_group)
		return

	var taken: Array[Vector3] = []
	if clear_node:
		taken.append(clear_node.global_position)
	var placed := 0
	for _attempt in car_count * 20:
		if placed >= car_count:
			break
		var lane: RoadLane = lanes.pick_random()
		var offset := randf() * lane.curve.get_baked_length()
		var pos := lane.to_global(lane.curve.sample_baked(offset))
		if taken.any(func(p): return p.distance_to(pos) < min_spacing):
			continue
		taken.append(pos)
		vehicles.add_actor(pos, lane)
		placed += 1
