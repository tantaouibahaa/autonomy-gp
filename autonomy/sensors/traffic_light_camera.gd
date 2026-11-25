class_name TrafficLightCamera
extends Node3D

## A wide-angle "camera" that reads traffic signals.
##
## Ground-truth assisted: instead of running a detector on pixels, it looks up
## the signal heads of the junction ahead and reads the colour they show, but
## only if the head is within range, inside the horizontal field of view, and
## not hidden behind a building. Like a driver, it can use the far-side head
## of the same phase when the near one is out of view. Optional dropout makes it miss frames, so the
## planner has to cope with "unknown".

enum { UNKNOWN = -1 }

@export var max_range := 90.0
@export var fov_deg := 180.0
@export_range(0.0, 1.0) var dropout := 0.03
## How long a reading is trusted after the light was last seen.
@export var memory := 0.6
@export var check_occlusion := true

var _last := {} # controller id -> {state, time}
var _time := 0.0


func _physics_process(delta: float) -> void:
	_time += delta


## State of the light that controls `lane` (an approach lane with "signal"
## meta): {state: TrafficLightController.Light or UNKNOWN, visible: bool}.
func observe(lane: RoadLane) -> Dictionary:
	var controller: TrafficLightController = lane.get_meta("signal")
	var arm: int = lane.get_meta("arm")
	var key := "%d:%d" % [controller.get_instance_id(), arm]
	var seen := false
	var state := UNKNOWN
	if randf() >= dropout:
		for head in controller.get_children():
			if not head.has_method("set_state"):
				continue
			# Heads of any arm in the same phase show the same colour, so
			# the far-side signal across the junction counts too.
			var head_arm := controller.arm_index_of(head.global_position)
			if controller.arm_group[head_arm] != controller.arm_group[arm]:
				continue
			if _can_see(head.global_position + Vector3.UP * 3.8):
				seen = true
				state = head.state
				break
	if seen:
		_last[key] = {state = state, time = _time}
	elif key in _last and _time - _last[key].time < memory:
		state = _last[key].state
	return {state = state, visible = seen}


func _can_see(target: Vector3) -> bool:
	var to := target - global_position
	if to.length() > max_range:
		return false
	var flat := Vector3(to.x, 0, to.z).normalized()
	var fwd := global_basis.z
	fwd.y = 0
	if rad_to_deg(fwd.normalized().angle_to(flat)) > fov_deg / 2.0:
		return false
	if check_occlusion:
		var q := PhysicsRayQueryParameters3D.create(global_position, target, 16)
		if not get_world_3d().direct_space_state.intersect_ray(q).is_empty():
			return false
	return true
