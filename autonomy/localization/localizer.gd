class_name Localizer
extends RefCounted

## Map-matched localization.
##
## The car's pose comes from the physics body (perfect odometry); this class
## snaps it onto the lane graph. With no route it uses TrafficNetwork.nearest()
## (global search). Along a route it projects the car onto the current route
## lane and steps to the next lane at the end, so lanes that overlap inside
## junctions can't confuse it. A lane change swaps the lane right away and
## keeps a sideways offset that fades out over the manoeuvre.

var network: TrafficNetwork
var route: Array = [] # steps from RoutePlanner.plan()
var goal_offset := 0.0
var idx := 0
var offset := 0.0
## Distance from the car to its matched lane point, meters.
var lane_error := 0.0

var _blend := Vector3.ZERO
var _blend_from := 0.0
var _blend_len := 1.0


func _init(net: TrafficNetwork) -> void:
	network = net


## Global map matching: {lane, offset, distance} of the nearest lane, or {}.
func locate(pos: Vector3, heading: Vector3, max_distance := 8.0) -> Dictionary:
	# nearest() may pick an opposite-direction lane; prefer one we face along.
	var best := {}
	var best_score := INF
	for lane in network.lanes:
		if lane.has_meta("turn"):
			continue
		var off := lane.curve.get_closest_offset(lane.to_local(pos))
		var hit: Array = network.sample(lane, off)
		var d: float = hit[0].distance_to(pos)
		if d > max_distance:
			continue
		var score: float = d + (0.0 if hit[1].dot(heading) > 0.5 else 100.0)
		if score < best_score:
			best_score = score
			best = {lane = lane, offset = off, distance = d}
	if best.is_empty():
		return network.nearest(pos, max_distance)
	return best


func set_route(steps: Array, goal_off: float, start_offset: float) -> void:
	route = steps
	goal_offset = goal_off
	idx = 0
	offset = start_offset
	_blend = Vector3.ZERO


func has_route() -> bool:
	return not route.is_empty()


func lane() -> RoadLane:
	return route[idx].lane if has_route() else null


func next_step() -> Dictionary:
	return route[idx + 1] if idx + 1 < route.size() else {}


func is_last_step() -> bool:
	return idx == route.size() - 1


## Project the car onto the route. Call every physics frame.
func update(pos: Vector3) -> void:
	if not has_route():
		return
	var l := lane()
	offset = l.curve.get_closest_offset(l.to_local(pos))
	while idx + 1 < route.size() and route[idx + 1].kind == "next" \
			and offset >= network.lane_length(l) - 0.3:
		idx += 1
		_blend = Vector3.ZERO # the lane-change blend belongs to the lane it started on
		l = lane()
		offset = maxf(l.curve.get_closest_offset(l.to_local(pos)), route[idx].entry)
	lane_error = (network.sample(l, offset)[0] + blend_at(offset) - pos).length()


## Sideways offset of the reference path at `lane_offset` on the current lane.
func blend_at(lane_offset: float) -> Vector3:
	if _blend == Vector3.ZERO:
		return Vector3.ZERO
	var t := clampf((lane_offset - _blend_from) / _blend_len, 0.0, 1.0)
	return _blend * (1.0 - smoothstep(0.0, 1.0, t))


func blending() -> bool:
	return _blend != Vector3.ZERO and offset < _blend_from + _blend_len


## Switch to the parallel `target` lane, merging over `length` meters.
## `pos` is the car's position. If `replan` is non-empty it becomes the new
## route (starting on `target`).
func change_lane(target: RoadLane, pos: Vector3, length: float, replan: Array) -> void:
	var here: Vector3 = network.sample(lane(), offset)[0] + blend_at(offset)
	if replan.is_empty():
		idx += 1 # the route's own "change" step
	else:
		route = replan
		idx = 0
	var off := target.curve.get_closest_offset(target.to_local(pos))
	offset = off
	_blend = here - network.sample(target, off)[0]
	_blend.y = 0.0
	_blend_from = off
	_blend_len = length
