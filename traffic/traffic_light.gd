@tool
extends Node3D

## A traffic signal on a pole, built from primitive meshes. The car signal
## faces local +Z (towards the cars it controls); the pedestrian WALK lamp
## faces local -X (across the road). TrafficLightController sets the state.

enum Light { RED, YELLOW, GREEN }

const LAMP_COLORS := [Color(1, 0.12, 0.08), Color(1, 0.7, 0.05), Color(0.1, 1, 0.35)]

@export var state: Light = Light.RED: set = set_state
@export var walk := false: set = set_walk
@export var pole_height := 4.2

var _lamps: Array[StandardMaterial3D] = []
var _walk_lamp: StandardMaterial3D


func _ready() -> void:
	add_to_group("traffic_lights")
	_build()
	set_state(state)
	set_walk(walk)


func set_state(value: Light) -> void:
	state = value
	for i in _lamps.size():
		_set_lamp(_lamps[i], LAMP_COLORS[i], i == state)


func set_walk(value: bool) -> void:
	walk = value
	if _walk_lamp:
		_set_lamp(_walk_lamp, Color(0.9, 0.95, 1) if walk else Color(1, 0.35, 0.05), true)


func _set_lamp(mat: StandardMaterial3D, color: Color, on: bool) -> void:
	mat.albedo_color = color if on else color.darkened(0.85)
	mat.emission_enabled = on
	mat.emission = color
	mat.emission_energy_multiplier = 3.0


func _build() -> void:
	var dark := StandardMaterial3D.new()
	dark.albedo_color = Color(0.12, 0.12, 0.13)
	dark.roughness = 0.6

	var pole := CylinderMesh.new()
	pole.top_radius = 0.07
	pole.bottom_radius = 0.09
	pole.height = pole_height
	pole.material = dark
	_add_mesh(pole, Vector3(0, pole_height / 2, 0))

	var housing := BoxMesh.new()
	housing.size = Vector3(0.38, 1.1, 0.3)
	housing.material = dark
	_add_mesh(housing, Vector3(0, pole_height - 0.45, 0.16))

	for i in 3:
		var mat := StandardMaterial3D.new()
		var lamp := SphereMesh.new()
		lamp.radius = 0.13
		lamp.height = 0.26
		lamp.material = mat
		_add_mesh(lamp, Vector3(0, pole_height - 0.1 - i * 0.34, 0.3))
		_lamps.append(mat)

	var ped_box := BoxMesh.new()
	ped_box.size = Vector3(0.25, 0.3, 0.3)
	ped_box.material = dark
	_add_mesh(ped_box, Vector3(-0.15, 2.4, 0))
	_walk_lamp = StandardMaterial3D.new()
	var ped_lamp := BoxMesh.new()
	ped_lamp.size = Vector3(0.04, 0.2, 0.2)
	ped_lamp.material = _walk_lamp
	_add_mesh(ped_lamp, Vector3(-0.28, 2.4, 0))


func _add_mesh(mesh: Mesh, pos: Vector3) -> void:
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	mi.position = pos
	add_child(mi) # not owned, so it is rebuilt on load instead of saved
