extends Node

## A tiny stand-in for the pedestrian spawner's sidewalk network, so the
## jaywalker scenario can reuse pedestrians/pedestrian.gd unchanged: ring 0 is
## the kerb the pedestrian waits on, ring 1 the far sidewalk they walk off
## along after crossing from `a` to `b`.

var rings: Array[PackedVector3Array] = []
var ring_length := PackedFloat32Array()
var links: Array = [[], []]
var cross_chance := 0.0
var jaywalk_chance := 0.0

var _a: Vector3
var _b: Vector3


func setup(a: Vector3, b: Vector3) -> void:
	_a = a
	_b = b
	var along := Vector3(1, 0, 0)
	rings = [PackedVector3Array([a, a + along]), PackedVector3Array([b - along * 100.0, b + along * 100.0])]
	ring_length = PackedFloat32Array([1.0, 200.0])


func crossing() -> Dictionary:
	return {s = 0.0, kind = "jaywalk", a = _a, b = _b, to_ring = 1, to_s = 100.0, controller = null}


func ring_point(r: int, s: float) -> Vector3:
	var pts: PackedVector3Array = rings[r]
	return pts[0].lerp(pts[1], clampf(s / ring_length[r], 0.0, 1.0))
