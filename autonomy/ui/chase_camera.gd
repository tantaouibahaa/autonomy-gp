class_name ChaseCamera
extends Camera3D

## Follows the ego car. C cycles: chase, high chase, top-down.

const VIEWS := [
	{back = 9.0, up = 3.6, look = 6.0},
	{back = 18.0, up = 11.0, look = 10.0},
	{back = 0.5, up = 55.0, look = 0.0},
]

var target: Node3D
var view := 0


func cycle() -> void:
	view = (view + 1) % VIEWS.size()


func _physics_process(delta: float) -> void:
	if target == null or not is_instance_valid(target):
		return
	var v: Dictionary = VIEWS[view]
	var fwd := target.global_basis.z
	fwd.y = 0
	fwd = fwd.normalized()
	var want: Vector3 = target.global_position - fwd * v.back + Vector3.UP * v.up
	var k := clampf(delta * 4.0, 0.0, 1.0)
	global_position = global_position.lerp(want, k) if global_position.distance_to(want) < 40.0 else want
	var look: Vector3 = target.global_position + fwd * v.look + Vector3.UP * 0.8
	if view == 2:
		look_at(look, fwd)
	else:
		look_at(look, Vector3.UP)
