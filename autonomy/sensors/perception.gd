class_name Perception
extends Node

## Turns raw sensor returns into tracked objects.
##
## Returns from every lidar (and the forward ShapeCast) are grouped into
## objects by the body they hit, a ground-truth data association that stands
## in for point-cloud clustering. Each object keeps a smoothed centroid and a
## velocity estimated from how that centroid moves between scans.

## Lidars and the ShapeCast to read, as NodePaths relative to this node.
@export var lidar_paths: Array[NodePath] = []
@export var shapecast_path: NodePath
## Seconds an object survives without returns before it is dropped.
@export var forget_after := 0.8
@export var velocity_smoothing := 0.25

## id -> {id, cls, pos, vel, points: Array[Vector3], last_seen, age}
var objects := {}
## Every return of the last frame: Array of {point, cls, id}.
var points: Array = []
## Closest return of the forward ShapeCast, or {} if clear.
var shapecast_hit := {}

var _lidars: Array[Lidar] = []
var _shapecast: ShapeCast3D
var _time := 0.0


func _ready() -> void:
	for p in lidar_paths:
		var l := get_node_or_null(p) as Lidar
		if l:
			_lidars.append(l)
	_shapecast = get_node_or_null(shapecast_path) as ShapeCast3D


func lidars() -> Array[Lidar]:
	return _lidars


## Scan every sensor and update the tracks. Called by the autopilot each frame.
func update(delta: float) -> void:
	_time += delta
	points.clear()
	var frame := {} # id -> Array[Vector3]
	var cls_of := {}
	for lidar in _lidars:
		lidar.scan()
		for h in lidar.hits:
			_add(frame, cls_of, h.collider, h.point, h.cls)
	shapecast_hit = {}
	if _shapecast:
		_shapecast.force_shapecast_update()
		var best := INF
		for i in _shapecast.get_collision_count():
			var c := _shapecast.get_collider(i)
			if c == null:
				continue
			var p := _shapecast.get_collision_point(i)
			_add(frame, cls_of, c, p, Lidar.classify(c))
			var d := p.distance_to(_shapecast.global_position)
			if d < best:
				best = d
				shapecast_hit = {point = p, distance = d, cls = Lidar.classify(c)}

	for id in frame:
		var pts: Array = frame[id]
		var c := Vector3.ZERO
		for p in pts:
			c += p
		c /= pts.size()
		var o: Dictionary = objects.get(id, {})
		if o.is_empty():
			o = {id = id, cls = cls_of[id], pos = c, vel = Vector3.ZERO, age = 0.0}
			objects[id] = o
		else:
			var dt: float = maxf(_time - o.last_seen, delta)
			var v: Vector3 = (c - o.pos) / dt
			v.y = 0.0
			# Centroids of partial views jump around; ignore implausible jumps.
			if v.length() < 45.0:
				o.vel = o.vel.lerp(v, velocity_smoothing)
			o.pos = c
			o.age += dt
		o.points = pts
		o.last_seen = _time
	for id in objects.keys():
		var o: Dictionary = objects[id]
		if _time - o.last_seen > forget_after:
			objects.erase(id)
		elif not id in frame:
			# Coast on the last velocity while the beams miss it.
			o.pos += o.vel * delta
			var shifted := []
			for p in o.points:
				shifted.append(p + o.vel * delta)
			o.points = shifted


func _add(frame: Dictionary, cls_of: Dictionary, collider: Object, p: Vector3, cls: String) -> void:
	var id := collider.get_instance_id()
	frame.get_or_add(id, []).append(p)
	cls_of[id] = cls
	points.append({point = p, cls = cls, id = id})
