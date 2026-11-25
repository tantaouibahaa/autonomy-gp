extends Node3D

## A parked car: a random Kenney car model with a static collision box.
## Counts as an obstacle (layer 4) as well as a vehicle (layer 2).

## Car models to pick from at random. Kenney models face +Z.
@export var models: Array[PackedScene] = []
@export var model_scale := 1.5

@onready var body: StaticBody3D = $Body


func _ready() -> void:
	if models.is_empty():
		return
	var model: Node3D = models.pick_random().instantiate()
	model.scale = Vector3.ONE * model_scale
	add_child(model)
	var box := AABB()
	var first := true
	for mi in model.find_children("*", "MeshInstance3D", true, false):
		var a: AABB = (global_transform.affine_inverse() * mi.global_transform) * mi.mesh.get_aabb()
		box = a if first else box.merge(a)
		first = false
	var shape := BoxShape3D.new()
	shape.size = box.size
	var cs: CollisionShape3D = body.get_node("CollisionShape3D")
	cs.shape = shape
	cs.position = box.get_center()
