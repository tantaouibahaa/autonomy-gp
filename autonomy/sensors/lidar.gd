class_name Lidar
extends Node3D

## A planar lidar built from a fan of RayCast3D beams. Each physics frame the
## beams are cast and the returns are stored in `hits`. The class of each return
## comes from the collision layer it hit (vehicles, pedestrians, obstacles,
## buildings): a ground-truth stand-in for a point-cloud segmentation network.

const CLASS_BY_LAYER := {2: "vehicle", 4: "pedestrian", 8: "obstacle", 16: "building"}

@export var beams := 48
## Horizontal field of view in degrees, centred on local +Z (the car's nose).
## 360 gives an evenly spaced full sweep.
@export var fov_deg := 120.0
@export var max_range := 60.0
## Layers 2-5: vehicles, pedestrians, obstacles, buildings.
@export_flags_3d_physics var mask := 2 | 4 | 8 | 16
@export var show_debug := false: set = set_show_debug

## Returns of the last scan: Array of {point, collider, cls, distance}.
var hits: Array = []

var _rays: Array[RayCast3D] = []
var _viz: MeshInstance3D
var _mesh: ImmediateMesh
var _mat_hit: StandardMaterial3D
var _mat_miss: StandardMaterial3D


func _ready() -> void:
	var body := _owner_body()
	for i in beams:
		var a: float
		if fov_deg >= 359.0:
			a = TAU * i / beams
		else:
			a = deg_to_rad(-fov_deg / 2.0 + fov_deg * i / maxf(beams - 1, 1))
		var ray := RayCast3D.new()
		ray.target_position = Vector3(sin(a), 0.0, cos(a)) * max_range
		ray.collision_mask = mask
		ray.enabled = false # cast on demand in scan()
		if body:
			ray.add_exception(body)
		add_child(ray)
		_rays.append(ray)
	set_show_debug(show_debug)


## Cast every beam now and refresh `hits`.
func scan() -> void:
	hits.clear()
	for ray in _rays:
		ray.force_raycast_update()
		if not ray.is_colliding():
			continue
		var collider := ray.get_collider()
		if collider == null:
			continue
		var p := ray.get_collision_point()
		hits.append({point = p, collider = collider, cls = classify(collider),
				distance = p.distance_to(global_position)})
	if _viz and _viz.visible:
		_draw()


static func classify(collider: Object) -> String:
	if not collider is CollisionObject3D:
		return "unknown"
	var layer: int = collider.collision_layer
	# Parked cars sit on both the vehicle and obstacle layers: static wins.
	for bit in [4, 8, 2, 16]:
		if layer & bit:
			return CLASS_BY_LAYER[bit]
	return "unknown"


func set_show_debug(value: bool) -> void:
	show_debug = value
	if not is_inside_tree():
		return
	if value and _viz == null:
		_mesh = ImmediateMesh.new()
		_viz = MeshInstance3D.new()
		_viz.mesh = _mesh
		_viz.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		_viz.top_level = true
		_mat_hit = _line_material(Color(1.0, 0.25, 0.15))
		_mat_miss = _line_material(Color(0.2, 0.9, 1.0, 0.25))
		add_child(_viz)
	if _viz:
		_viz.visible = value


func _draw() -> void:
	_viz.global_transform = Transform3D.IDENTITY
	_mesh.clear_surfaces()
	var origin := global_position
	var hit_set := {}
	for h in hits:
		hit_set[h.point] = true
	_mesh.surface_begin(Mesh.PRIMITIVE_LINES, _mat_miss)
	for ray in _rays:
		if not ray.is_colliding():
			_mesh.surface_add_vertex(origin)
			_mesh.surface_add_vertex(ray.to_global(ray.target_position))
	_mesh.surface_add_vertex(origin) # a surface needs at least one line
	_mesh.surface_add_vertex(origin)
	_mesh.surface_end()
	if hits.is_empty():
		return
	_mesh.surface_begin(Mesh.PRIMITIVE_LINES, _mat_hit)
	for h in hits:
		_mesh.surface_add_vertex(origin)
		_mesh.surface_add_vertex(h.point)
		# small cross at the return
		_mesh.surface_add_vertex(h.point + Vector3(0.3, 0, 0))
		_mesh.surface_add_vertex(h.point - Vector3(0.3, 0, 0))
		_mesh.surface_add_vertex(h.point + Vector3(0, 0, 0.3))
		_mesh.surface_add_vertex(h.point - Vector3(0, 0, 0.3))
	_mesh.surface_end()


func _line_material(color: Color) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.albedo_color = color
	if color.a < 1.0:
		m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	m.no_depth_test = false
	return m


func _owner_body() -> CollisionObject3D:
	var n := get_parent()
	while n:
		if n is CollisionObject3D:
			return n
		n = n.get_parent()
	return null
