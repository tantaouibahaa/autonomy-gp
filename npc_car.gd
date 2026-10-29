extends "res://road_demos/demo_resources/actors/road_actor.gd"

## NPC traffic car: the demo RoadActor lane-following logic with a random
## Kenney Car Kit model on top.

## Car models to pick from at random. Kenney models face +Z.
@export var models: Array[PackedScene] = []
@export var model_scale := 1.5


func _ready() -> void:
	if not models.is_empty():
		var model: Node3D = models.pick_random().instantiate()
		model.scale = Vector3.ONE * model_scale
		model.rotation.y = PI # RoadActors drive along -Z
		add_child(model)
	super()
