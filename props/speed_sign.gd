@tool
extends Node3D

## Round speed limit sign facing local +Z. `limit` is in km/h.

@export var limit := 50: set = set_limit

var _label: Label3D


func _ready() -> void:
	add_to_group("speed_signs")
	var grey := StandardMaterial3D.new()
	grey.albedo_color = Color(0.55, 0.56, 0.58)
	var pole := CylinderMesh.new()
	pole.top_radius = 0.04
	pole.bottom_radius = 0.04
	pole.height = 2.6
	pole.material = grey
	_add(pole, Transform3D(Basis.IDENTITY, Vector3(0, 1.3, 0)))

	var white := StandardMaterial3D.new()
	white.albedo_color = Color(0.95, 0.95, 0.95)
	var disc := CylinderMesh.new()
	disc.top_radius = 0.38
	disc.bottom_radius = 0.38
	disc.height = 0.03
	disc.material = white
	_add(disc, Transform3D(Basis(Vector3.RIGHT, PI / 2), Vector3(0, 2.6, 0.06)))

	var red := StandardMaterial3D.new()
	red.albedo_color = Color(0.85, 0.08, 0.08)
	var ring := TorusMesh.new()
	ring.inner_radius = 0.3
	ring.outer_radius = 0.4
	ring.material = red
	_add(ring, Transform3D(Basis(Vector3.RIGHT, PI / 2).scaled(Vector3(1, 1, 0.3)), Vector3(0, 2.6, 0.08)))

	_label = Label3D.new()
	_label.font_size = 96
	_label.pixel_size = 0.004
	_label.modulate = Color.BLACK
	_label.outline_size = 0
	_label.position = Vector3(0, 2.6, 0.085)
	add_child(_label)
	set_limit(limit)


func set_limit(value: int) -> void:
	limit = value
	if _label:
		_label.text = str(limit)


func _add(mesh: Mesh, xf: Transform3D) -> void:
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	mi.transform = xf
	add_child(mi) # not owned, so it is rebuilt on load instead of saved
