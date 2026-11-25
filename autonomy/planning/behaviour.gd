class_name Behaviour
extends RefCounted

## Behaviour layer: a finite state machine on top of a speed planner.
##
## Every frame it gathers "stop constraints" (a point on the path where the
## car must be going no faster than some speed): the speed limit and curves,
## red lights, yield and merge points, the goal, and anything the sensors
## see in the corridor ahead. The tightest constraint gives the target speed
## and decides the state:
##   CRUISE           nothing but the speed limit / curves
##   FOLLOW           adaptive cruise behind a moving vehicle
##   STOP_AT_LIGHT    red (or yellow we can stop for, or unseen) signal
##   YIELD            roundabout entry or merge with traffic in conflict
##   EMERGENCY_BRAKE  something in the way that needs a hard stop
##   LANE_CHANGE      blending into a neighbour lane
##   WAIT             stopped behind a static obstacle we can't get around yet
##   ARRIVED          goal reached

enum State { IDLE, CRUISE, FOLLOW, STOP_AT_LIGHT, YIELD, EMERGENCY_BRAKE, LANE_CHANGE, WAIT, ARRIVED }

const STATE_NAMES := ["IDLE", "CRUISE", "FOLLOW", "STOP_AT_LIGHT", "YIELD", "EMERGENCY_BRAKE", "LANE_CHANGE", "WAIT", "ARRIVED"]

# Vehicle and comfort parameters.
var half_length := 2.0
var half_width := 0.95
var corridor_margin := 0.55
var comfort_decel := 2.2
var yellow_max_decel := 4.0
var emergency_threshold := 3.5
var static_gap := 2.5
var ped_gap := 4.0
var time_gap := 1.3
var min_gap := 3.0
## Fraction of the speed limit to aim for, leaving room for overshoot.
var limit_factor := 0.96
## Seconds of look-ahead for crossing pedestrians and vehicles.
var ped_horizon := 2.5
var vehicle_horizon := 1.5

var state: int = State.IDLE
var reason := ""
## Last light reading, for the HUD: {state, distance} or {}.
var light := {}
## Object currently binding the speed, for the HUD/debug.
var lead := {}
var target_speed := 0.0
## Deceleration the binding constraint needs, m/s^2 (0 if none).
var required_decel := 0.0
var lane_change_request := "" # "", "route", "obstacle", "overtake"
## Change lanes to pass vehicles slower than we want to go (race mode).
var overtake := false

var planner: RoutePlanner
var _upstream_cache := {}


func state_name() -> String:
	return STATE_NAMES[state]


## Decide the target speed. Returns nothing; read target_speed, state, etc.
func decide(pos: Vector3, fwd: Vector3, v: float, path: RefPath, objects: Dictionary,
		camera: TrafficLightCamera, loc: Localizer, dt: float) -> void:
	var cons := [] # {s, v_end, kind, label, obj}
	light = {}
	lead = {}
	lane_change_request = ""

	# 1. Speed limits and curves along the path.
	var cruise := INF
	for i in path.pts.size():
		var si := path.s[i]
		var vm := path.vmax[i] * limit_factor
		if si <= 0.0:
			if si > -1.5:
				cruise = minf(cruise, vm)
			continue
		cruise = minf(cruise, sqrt(vm * vm + 2.0 * comfort_decel * si))
	if cruise == INF:
		cruise = 5.0

	# 2. Map events: signals, yields, merges, goal, end of path.
	for ev in path.events:
		match ev.kind:
			"signal":
				if ev.s < -1.5:
					continue
				var obs := camera.observe(ev.lane)
				if light.is_empty():
					light = {state = obs.state, distance = ev.s, visible = obs.visible}
				if _must_stop_for_light(obs.state, ev.s, v):
					cons.append({s = ev.s - 0.4, v_end = 0.0, kind = "light", label = "signal"})
			"yield":
				if ev.s > -0.5 and ev.s < 30.0 and _yield_conflict(ev, path, objects):
					cons.append({s = ev.s - 1.0, v_end = 0.0, kind = "yield", label = "roundabout"})
			"merge":
				if ev.s > -0.5 and ev.s < 60.0 and _merge_conflict(ev, path, objects, v, loc):
					cons.append({s = ev.s - 1.5, v_end = 0.0, kind = "yield", label = "merge"})
			"goal":
				cons.append({s = ev.s, v_end = 0.0, kind = "goal", label = "goal"})
			"end":
				cons.append({s = ev.s - 2.0, v_end = 0.0, kind = "end", label = ev.get("reason", "end")})

	# 3. Objects in or entering the corridor ahead.
	var corridor := half_width + corridor_margin
	var obstacle_ahead := {}
	for id in objects:
		var o: Dictionary = objects[id]
		if o.cls == "building":
			continue
		var hit := _closest_in_corridor(o, pos, fwd, path, corridor)
		var predicted := false
		if hit.is_empty() and (o.cls == "pedestrian" or o.cls == "vehicle"):
			hit = _predict_entry(o, path, corridor, v)
			predicted = not hit.is_empty()
		if hit.is_empty():
			continue
		var along: float = hit.s
		var v_obj := 0.0
		if o.cls == "vehicle" and not predicted:
			var d: Vector3 = path.dirs[hit.index]
			v_obj = maxf(o.vel.dot(d), 0.0)
			if v_obj < 0.5:
				v_obj = 0.0
		var gap := min_gap
		match o.cls:
			"pedestrian":
				gap = ped_gap
			"obstacle":
				gap = static_gap
			_:
				gap = min_gap + time_gap * v_obj
		var c := {s = along - half_length - gap, v_end = v_obj, kind = o.cls,
				label = o.cls + (" (crossing)" if predicted else ""), obj = o, dist = along - half_length}
		cons.append(c)
		if o.cls == "obstacle" and along < 60.0 and (obstacle_ahead.is_empty() or along < obstacle_ahead.dist):
			obstacle_ahead = c

	# 4. Pick the binding constraint.
	target_speed = cruise
	var binding := {}
	for c in cons:
		var vt: float
		if c.s > 0.0:
			vt = sqrt(c.v_end * c.v_end + 2.0 * comfort_decel * c.s)
		else:
			# Inside the desired gap: fall back below the lead's speed.
			vt = c.v_end * clampf(1.0 + c.s / 8.0, 0.0, 1.0)
		if vt < target_speed:
			target_speed = vt
			binding = c
	required_decel = 0.0
	if not binding.is_empty() and v > binding.v_end:
		required_decel = (v * v - binding.v_end * binding.v_end) / (2.0 * maxf(binding.s, 0.25))

	# 5. Lane changes: around static obstacles, or because the route says so.
	if loc.blending():
		pass
	elif not obstacle_ahead.is_empty():
		lane_change_request = "obstacle"
	elif overtake and binding.get("kind", "") == "vehicle" and binding.dist < 60.0 \
			and binding.v_end < cruise - 3.0:
		lane_change_request = "overtake"
	elif loc.next_step().get("kind", "") == "change":
		lane_change_request = "route"

	# 6. State.
	var reached_goal := false
	for ev in path.events:
		if ev.kind == "goal" and ev.s < 1.5:
			reached_goal = true
	if reached_goal and v < 0.6:
		state = State.ARRIVED
		reason = "goal reached"
		target_speed = 0.0
		return
	lead = binding.get("obj", {})
	if not binding.is_empty() and binding.kind in ["pedestrian", "obstacle", "vehicle"] \
			and required_decel > emergency_threshold and v > 1.0:
		state = State.EMERGENCY_BRAKE
	elif loc.blending():
		state = State.LANE_CHANGE
	elif binding.is_empty():
		state = State.CRUISE
	else:
		match binding.kind:
			"light":
				state = State.STOP_AT_LIGHT
			"yield":
				state = State.YIELD
			"vehicle":
				state = State.FOLLOW
			"pedestrian":
				state = State.EMERGENCY_BRAKE if required_decel > comfort_decel else State.WAIT
			"obstacle", "end":
				state = State.WAIT if v < 3.0 else State.CRUISE
			_:
				state = State.CRUISE
	reason = binding.get("label", "speed limit")


func _must_stop_for_light(light_state: int, d: float, v: float) -> bool:
	match light_state:
		TrafficLightController.Light.GREEN:
			return false
		TrafficLightController.Light.RED:
			return true
		_: # yellow or unseen: stop if we comfortably can
			if d <= 0.0:
				return light_state != TrafficLightController.Light.YELLOW
			return v * v / (2.0 * d) <= yellow_max_decel


## Nearest return of `o` inside the corridor ahead: {s, index, lateral} or {}.
func _closest_in_corridor(o: Dictionary, pos: Vector3, fwd: Vector3, path: RefPath, corridor: float) -> Dictionary:
	var best := {}
	for p in o.points:
		var rel: Vector3 = p - pos
		rel.y = 0.0
		var r := rel.length()
		if rel.dot(fwd) < 0.0 or r > path.length + 2.0:
			continue
		# A point within `corridor` of path point i is at least r - corridor
		# along the path, so skip everything before that.
		var pr := path.project(p, r - corridor - 2.0, r * 1.6 + 6.0)
		if pr.is_empty() or pr.lateral > corridor:
			continue
		if best.is_empty() or pr.s < best.s:
			best = pr
	return best


## Frenet-frame prediction for something moving across the path.
func _predict_entry(o: Dictionary, path: RefPath, corridor: float, ego_v: float) -> Dictionary:
	var speed: float = o.vel.length()
	var ped: bool = o.cls == "pedestrian"
	if speed < (0.4 if ped else 1.0) or o.age < 0.15:
		return {}
	var pr := path.project(o.pos, 0.0, path.length)
	if pr.is_empty() or pr.lateral > 14.0:
		return {}
	var i: int = pr.index
	var dir: Vector3 = path.dirs[i]
	var normal := dir.cross(Vector3.UP)
	var d0: float = (o.pos - path.pts[i]).dot(normal)
	var vd: float = o.vel.dot(normal)
	var vs: float = o.vel.dot(dir)
	# Only things closing in on the path sideways.
	if absf(vd) < (0.4 if ped else 1.0) or signf(vd) == signf(d0):
		return {}
	var horizon := ped_horizon if ped else vehicle_horizon
	var margin := 0.3 if ped else 0.0
	var t := 0.0
	while t <= horizon:
		var d := d0 + vd * t
		if absf(d) < corridor + margin:
			var s_at: float = pr.s + vs * t
			# Irrelevant if we'll be long past (or not there yet for far things).
			if s_at < 0.0:
				return {}
			if ped or s_at < ego_v * (t + 1.5) + 8.0:
				return {s = s_at, index = clampi(i + int(vs * t), 0, path.pts.size() - 1), lateral = absf(d)}
			return {}
		t += 0.25
	return {}


## Any vehicle in, or about to be in, the first stretch of our path through the roundabout?
func _yield_conflict(ev: Dictionary, path: RefPath, objects: Dictionary) -> bool:
	var centre: Vector3 = ev.junction.global_position
	for id in objects:
		var o: Dictionary = objects[id]
		if o.cls != "vehicle" or o.pos.distance_to(centre) > 34.0:
			continue
		# Our own leader still on the approach lane isn't a conflict.
		var own := path.project(o.pos, -6.0, ev.s - 0.5)
		if not own.is_empty() and own.lateral < 2.0:
			continue
		var t := 0.0
		while t <= 3.0:
			var q: Vector3 = o.pos + o.vel * t
			var pr := path.project(q, ev.s - 2.0, ev.s + 22.0)
			if not pr.is_empty() and pr.lateral < 3.2:
				return true
			t += 0.5
	return false


## Gap acceptance at a merge: is someone on a feeding lane going to reach the
## merge point within a couple of seconds of us?
func _merge_conflict(ev: Dictionary, path: RefPath, objects: Dictionary, v: float, loc: Localizer) -> bool:
	var net := loc.network
	var t_e: float = ev.s / maxf(v, 3.0)
	var key: RoadLane = ev.into
	if not key in _upstream_cache:
		_upstream_cache[key] = _upstream(net, ev.others, 160.0)
	var feeders: Array = _upstream_cache[key]
	for id in objects:
		var o: Dictionary = objects[id]
		if o.cls != "vehicle":
			continue
		for f in feeders:
			var lane: RoadLane = f.lane
			var off := lane.curve.get_closest_offset(lane.to_local(o.pos))
			var sp: Array = net.sample(lane, off)
			if sp[0].distance_to(o.pos) > 2.6:
				continue
			var d_o: float = net.lane_length(lane) - off + f.extra
			var v_o: float = maxf(o.vel.dot(sp[1]), 0.0)
			if d_o < 10.0:
				return true # alongside us at the merge point
			var t_o: float = d_o / maxf(v_o, 0.5)
			if t_o > t_e - 1.2 and t_o < t_e + 2.5:
				return true
			break
	return false


func _upstream(net: TrafficNetwork, lanes: Array, reach: float) -> Array:
	var out := []
	var frontier := []
	for l in lanes:
		frontier.append({lane = l, extra = 0.0})
	var seen := {}
	while not frontier.is_empty():
		var f: Dictionary = frontier.pop_back()
		if f.lane in seen:
			continue
		seen[f.lane] = true
		out.append(f)
		var extra: float = f.extra + net.lane_length(f.lane)
		if extra > reach:
			continue
		for lane in planner.predecessors(f.lane):
			frontier.append({lane = lane, extra = extra})
	return out
