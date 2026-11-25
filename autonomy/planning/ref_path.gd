class_name RefPath
extends RefCounted

## The reference path the car tracks: the route ahead sampled every meter,
## with the lane-change blend applied, plus the events the planner reacts to
## (signals, yield and merge points, goal, end of path).

const STEP := 1.0

var pts := PackedVector3Array()
var dirs := PackedVector3Array()
var s := PackedFloat32Array() # distance from the car's matched point
var vmax := PackedFloat32Array()
var lanes: Array = []
var offs := PackedFloat32Array()
## Array of {kind, s, ...}: "signal" (lane), "yield" (lane, junction),
## "merge" (lane, into, others), "goal", "end".
var events: Array = []
var length := 0.0


## Build up to `ahead` meters of path from the localizer's current state.
static func build(loc: Localizer, planner: RoutePlanner, ahead: float, lat_accel: float, back := 4.0) -> RefPath:
	var p := RefPath.new()
	var net := loc.network
	var k := loc.idx
	var o := maxf(loc.offset - back, loc.route[k].entry if k > 0 else 0.0)
	var s0 := o - loc.offset # negative: a few meters behind the car
	var dist := s0
	while true:
		var step: Dictionary = loc.route[k]
		var lane: RoadLane = step.lane
		var len := net.lane_length(lane)
		var last := k == loc.route.size() - 1
		# Past the goal the path runs on a bit, so the car still sees what is
		# parked just beyond it.
		var end := minf(loc.goal_offset + 15.0, len) if last else len
		var limit := net.speed_limit(lane)
		if lane.has_meta("signal") and lane.get_meta("stop_offset") > o - 0.5:
			p.events.append({kind = "signal", s = dist + lane.get_meta("stop_offset") - o, lane = lane})
		if lane.has_meta("yield_at") and len > o:
			p.events.append({kind = "yield", s = dist + len - o, lane = lane, junction = lane.get_meta("yield_at")})
		var x := o
		while x <= end and dist <= ahead:
			var sp: Array = net.sample(lane, x)
			var pt: Vector3 = sp[0]
			if k == loc.idx:
				pt += loc.blend_at(x)
			p.pts.append(pt)
			p.dirs.append(sp[1])
			p.s.append(dist)
			p.vmax.append(limit)
			p.lanes.append(lane)
			p.offs.append(x)
			x += STEP
			dist += STEP
		if dist > ahead:
			break
		dist -= x - end # land exactly on the lane end
		if last:
			p.events.append({kind = "goal", s = dist - (end - loc.goal_offset)})
			break
		var nxt: Dictionary = loc.route[k + 1]
		if nxt.kind != "next":
			p.events.append({kind = "end", s = dist, reason = "lane change"})
			break
		var into: RoadLane = nxt.lane
		var others := planner.predecessors(into).filter(func(l): return l != lane and not l.has_meta("turn"))
		if not others.is_empty() and not lane.has_meta("turn") and not into.has_meta("turn"):
			# Lanes that merge with a taper overlap before they meet: the
			# conflict starts where the other lane comes within a car width.
			var overlap := _overlap(net, lane, others)
			p.events.append({kind = "merge", s = dist - overlap, lane = lane, into = into,
					others = others, overlap = overlap})
		k += 1
		o = nxt.entry
	p.length = dist
	p._fix_dirs()
	p._curvature_limits(lat_accel)
	return p


static func _overlap(net: TrafficNetwork, lane: RoadLane, others: Array) -> float:
	var len := net.lane_length(lane)
	var back := 0.0
	while back < 80.0 and back < len:
		var pt: Vector3 = net.sample(lane, len - back)[0]
		var near := INF
		for o: RoadLane in others:
			var q := o.to_global(o.curve.sample_baked(o.curve.get_closest_offset(o.to_local(pt))))
			near = minf(near, Vector2(q.x - pt.x, q.z - pt.z).length())
		if near > 3.8:
			break
		back += 1.0
	return back


func _fix_dirs() -> void:
	# Lane-change blends bend the path, so take directions from the points.
	for i in pts.size():
		var a := pts[maxi(i - 1, 0)]
		var b := pts[mini(i + 1, pts.size() - 1)]
		var d := b - a
		d.y = 0
		if d.length() > 0.01:
			dirs[i] = d.normalized()


func _curvature_limits(lat_accel: float) -> void:
	var n := pts.size()
	for i in n:
		var a := dirs[maxi(i - 3, 0)]
		var b := dirs[mini(i + 3, n - 1)]
		var span := s[mini(i + 3, n - 1)] - s[maxi(i - 3, 0)]
		if span < 1.0:
			continue
		var kappa := a.angle_to(b) / span
		if kappa > 0.002:
			vmax[i] = minf(vmax[i], sqrt(lat_accel / kappa))


## Index of the path point nearest to `pos` (horizontal), searching all points.
func nearest_index(pos: Vector3) -> int:
	var best := 0
	var best_d := INF
	for i in pts.size():
		var d := Vector2(pts[i].x - pos.x, pts[i].z - pos.z).length_squared()
		if d < best_d:
			best_d = d
			best = i
	return best


## Point at path distance `at` (clamped to the path).
func point_at(at: float) -> Vector3:
	if pts.is_empty():
		return Vector3.ZERO
	var i := clampi(int((at - s[0]) / STEP), 0, pts.size() - 1)
	return pts[i]


## Horizontal distance from `pos` to the path and the path distance there:
## {lateral, s, index}. Only points with s in [s_min, s_max] are considered.
func project(pos: Vector3, s_min := -INF, s_max := INF) -> Dictionary:
	var best := -1
	var best_d := INF
	for i in pts.size():
		if s[i] < s_min or s[i] > s_max:
			continue
		var d := Vector2(pts[i].x - pos.x, pts[i].z - pos.z).length_squared()
		if d < best_d:
			best_d = d
			best = i
	if best < 0:
		return {}
	# refine against the segment to the better neighbour
	var j := best + 1 if best + 1 < pts.size() else best - 1
	var lateral := sqrt(best_d)
	var along := s[best]
	if j >= 0 and j < pts.size():
		var q := Geometry3D.get_closest_point_to_segment(Vector3(pos.x, 0, pos.z),
				Vector3(pts[best].x, 0, pts[best].z), Vector3(pts[j].x, 0, pts[j].z))
		lateral = Vector2(q.x - pos.x, q.z - pos.z).length()
		along = s[best] + (q - Vector3(pts[best].x, 0, pts[best].z)).length() * signf(s[j] - s[best])
	return {lateral = lateral, s = along, index = best}
