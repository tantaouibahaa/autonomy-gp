class_name TrafficLightController
extends Node3D

## Runs the signal cycle for one junction. Must be a child of the junction's
## RoadContainer; its arms are read from the container's edge RoadPoints.
##
## Cycle: arms along the junction's local Z axis go green, then arms along
## local X, then every crosswalk gets WALK (a pedestrian scramble) while all
## cars wait. Opposite arms share a green, so NPCs only go straight or right
## there (see TrafficNetwork); an arm that has a green alone may also turn left.

signal phase_changed(phase: int)

enum Light { RED, YELLOW, GREEN }

@export var green_time := 12.0
@export var yellow_time := 3.0
@export var all_red_time := 1.5
@export var walk_time := 14.0
@export var walk_flash_time := 5.0
## Start somewhere random in the cycle so junctions don't switch in sync.
@export var random_start := true
## Distance from the junction centre to the stop line, in meters.
@export var stop_line_distance := 22.0

## Unit vectors from the junction centre out along each arm.
var arms: Array[Vector3] = []
## Which green phase (0 or 1) each arm belongs to.
var arm_group: Array[int] = []

var _phases: Array[Dictionary] = []
var _phase := 0
var _time_left := 0.0


func _ready() -> void:
	add_to_group("signal_controllers")
	_read_arms()
	for group in 2:
		if group in arm_group:
			_phases.append({time = green_time, green = group})
			_phases.append({time = yellow_time, yellow = group})
			_phases.append({time = all_red_time})
	_phases.append({time = walk_time, walk = true})
	_phases.append({time = walk_flash_time, flash = true})
	_phases.append({time = all_red_time})
	_phase = randi() % _phases.size() if random_start else 0
	_time_left = _phases[_phase].time * (randf() if random_start else 1.0)
	_apply()


func _process(delta: float) -> void:
	_time_left -= delta
	if _time_left <= 0.0:
		_phase = (_phase + 1) % _phases.size()
		_time_left += _phases[_phase].time
		_apply()
	elif is_walk_flashing():
		_update_lights()


## State of the signal facing cars that approach along `arm`.
func get_state(arm: int) -> Light:
	var p := _phases[_phase]
	if p.get("green", -1) == arm_group[arm]:
		return Light.GREEN
	if p.get("yellow", -1) == arm_group[arm]:
		return Light.YELLOW
	return Light.RED


## True while pedestrians may start crossing (not while the WALK light flashes).
func is_walk() -> bool:
	return _phases[_phase].get("walk", false)


func is_walk_flashing() -> bool:
	return _phases[_phase].get("flash", false)


## Seconds until the current phase changes.
func time_left() -> float:
	return _time_left


## Index of the arm whose direction best matches `world_pos` seen from the centre.
func arm_index_of(world_pos: Vector3) -> int:
	var to_pos := world_pos - global_position
	to_pos.y = 0
	to_pos = to_pos.normalized()
	var best := 0
	for i in arms.size():
		if arms[i].dot(to_pos) > arms[best].dot(to_pos):
			best = i
	return best


## True when this arm is alone in its green phase (e.g. a T-junction's branch).
func arm_has_own_phase(arm: int) -> bool:
	return arm_group.count(arm_group[arm]) == 1


func _read_arms() -> void:
	var junction: Node3D = get_parent()
	var axis_z: Vector3 = junction.global_basis.z.normalized()
	for path in junction.edge_rp_locals:
		var rp: Node3D = junction.get_node(path)
		var dir := rp.global_position - junction.global_position
		dir.y = 0
		if dir.length() < 5.0:
			continue # closed stub inside the junction, not an arm
		arms.append(dir.normalized())
		arm_group.append(0 if absf(dir.normalized().dot(axis_z)) > 0.7 else 1)


func _apply() -> void:
	_update_lights()
	phase_changed.emit(_phase)


func _update_lights() -> void:
	for light in get_children():
		if light.has_method("set_state"):
			var arm := arm_index_of(light.global_position)
			light.set_state(get_state(arm))
			light.set_walk(is_walk() or (is_walk_flashing() and fmod(_time_left, 1.0) > 0.5))
