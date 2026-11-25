class_name Autopilot
extends Node

## The self-driving stack, run once per physics frame:
##   perception -> localization -> planning (route, behaviour, lane changes)
##   -> control (pure pursuit + PID)
## The car (ego_car.gd) turns the resulting acceleration and steering into
## engine force, brake and wheel angle.

signal arrived
signal route_changed

@export var perception_path: NodePath
@export var camera_path: NodePath
@export var max_accel := 2.5
@export var max_decel := 8.0
@export var max_steer := 0.6
@export var lateral_accel := 3.0
## Comfort jerk limit (m/s^3) on the commanded acceleration; emergencies use 40.
@export var jerk_limit := 6.0

var network: TrafficNetwork
var planner: RoutePlanner
var loc: Localizer
var behaviour := Behaviour.new()
var pursuit := PurePursuit.new()
var speed_pid := PID.new(1.4, 0.25, 0.0, 3.0)
var path: RefPath
var perception: Perception
var camera: TrafficLightCamera

var goal := {} # {lane, offset}
## Output of the last step: accel (m/s^2), steer (rad).
var accel := 0.0
var steer := 0.0
var active := false
var has_arrived := false
## Seconds the car has been unable to follow its route at all.
var replans := 0

var _car: VehicleBody3D
var _off_route_time := 0.0
var _lane_change_cooldown := 0.0


func _ready() -> void:
	_car = get_parent() as VehicleBody3D
	perception = get_node(perception_path)
	camera = get_node(camera_path)


func setup(net: TrafficNetwork) -> void:
	network = net
	planner = RoutePlanner.new(net)
	loc = Localizer.new(net)
	behaviour.planner = planner


## Plan a route from where the car is to (lane, offset). False if unreachable.
func set_goal(lane: RoadLane, offset: float) -> bool:
	goal = {lane = lane, offset = offset}
	has_arrived = false
	return _replan()


func clear_goal() -> void:
	goal = {}
	loc.route = []
	path = null


func _replan() -> bool:
	if goal.is_empty():
		return false
	var here := loc.locate(_car.global_position, _car.global_basis.z)
	if here.is_empty():
		return false
	var steps := planner.plan(here.lane, here.offset, goal.lane, goal.offset)
	if steps.is_empty():
		return false
	loc.set_route(steps, goal.offset, here.offset)
	replans += 1
	route_changed.emit()
	return true


func speed() -> float:
	return _car.linear_velocity.dot(_car.global_basis.z)


## Run the whole stack. Perception always runs (for the HUD and debug view);
## the rest only while a route is set.
func step(delta: float) -> void:
	perception.update(delta)
	if goal.is_empty() or not loc.has_route():
		accel = -3.0
		steer = 0.0
		behaviour.state = Behaviour.State.IDLE
		return
	var pos := _car.global_position
	var fwd := _car.global_basis.z
	fwd.y = 0
	fwd = fwd.normalized()
	var v := speed()

	# Localization.
	loc.update(pos)
	if loc.lane_error > 4.0:
		_off_route_time += delta
		if _off_route_time > 1.0:
			_off_route_time = 0.0
			_replan()
	else:
		_off_route_time = 0.0

	# Planning.
	var ahead := clampf(v * 6.0 + 45.0, 60.0, 170.0)
	path = RefPath.build(loc, planner, ahead, lateral_accel)
	behaviour.decide(pos, fwd, v, path, perception.objects, camera, loc, delta)
	_lane_change_cooldown -= delta
	if behaviour.lane_change_request != "" and _lane_change_cooldown <= 0.0:
		_try_lane_change(behaviour.lane_change_request, pos, v)

	if behaviour.state == Behaviour.State.ARRIVED and not has_arrived:
		has_arrived = true
		arrived.emit()

	# Control: pure pursuit for steering, PID on speed for acceleration.
	var rear := pos - fwd * 0.99
	var want_steer := pursuit.steer(rear, fwd, v, path, 0.0)
	# Never ask for more lateral acceleration than the tyres can give.
	if absf(v) > 3.0:
		var cap := atan(7.0 * pursuit.wheelbase / (v * v))
		want_steer = clampf(want_steer, -cap, cap)
	steer = clampf(want_steer, -max_steer, max_steer)

	var target := behaviour.target_speed
	var a: float
	if target < 0.05 and v < 0.5:
		a = -3.0 # hold at standstill
		speed_pid.reset()
	else:
		a = speed_pid.step(target - v, delta)
		if behaviour.required_decel > 0.5 and target < v:
			a = minf(a, -behaviour.required_decel * 1.05)
	a = clampf(a, -max_decel, max_accel)
	var emergency := behaviour.state == Behaviour.State.EMERGENCY_BRAKE
	var j := 40.0 if emergency or a < accel - 4.0 else jerk_limit
	accel = move_toward(accel, a, j * delta)


## Change into a neighbour lane if its gap is free.
func _try_lane_change(kind: String, pos: Vector3, v: float) -> void:
	var lane := loc.lane()
	if not planner.can_change_on(lane):
		return
	var length := clampf(v * 3.0, 14.0, 40.0)
	var candidates := []
	if kind == "route":
		candidates = [loc.next_step().lane]
	else:
		for n in network.neighbours(lane):
			candidates.append(n.lane)
	for target: RoadLane in candidates:
		var off := target.curve.get_closest_offset(target.to_local(pos))
		if network.lane_length(target) - off < length + 8.0:
			continue
		if not _gap_free(target, off, v, kind != "route"):
			continue
		var replan := []
		if kind != "route":
			replan = planner.plan(target, off, goal.lane, goal.offset)
			if replan.is_empty():
				continue
		loc.change_lane(target, pos, length, replan)
		_lane_change_cooldown = 3.0
		route_changed.emit()
		return


## Is `lane` clear around offset `off`? Uses every sensor return near it.
func _gap_free(lane: RoadLane, off: float, v: float, need_clear_ahead: bool) -> bool:
	var ahead := maxf(15.0, v * 1.5) + (35.0 if need_clear_ahead else 0.0)
	var samples := []
	var x := off - 45.0
	while x <= off + ahead:
		if x >= 0.0 and x <= network.lane_length(lane):
			samples.append([x - off, network.sample(lane, x)[0]])
		x += 2.0
	for id in perception.objects:
		var o: Dictionary = perception.objects[id]
		if o.cls == "building":
			continue
		for p in o.points:
			for smp in samples:
				var q: Vector3 = smp[1]
				if Vector2(q.x - p.x, q.z - p.z).length() > 1.9:
					continue
				var rel: float = smp[0]
				if rel > -8.0:
					return false # beside or ahead of us in that lane
				# behind: fine only if it isn't closing in fast
				var closing: float = o.vel.length() - v
				if closing > 0.0 and -rel / closing < 3.5:
					return false
				break
	return true
