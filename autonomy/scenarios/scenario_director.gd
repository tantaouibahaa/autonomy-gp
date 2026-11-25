class_name ScenarioDirector
extends Node

## Runs one scenario: sets up its special actors (a forced signal phase, a
## jaywalker, extra traffic), times it, and scores it with Metrics.

signal finished(result: Dictionary)

const PEDESTRIAN := preload("res://pedestrians/pedestrian.tscn")
const ScriptedWalk := preload("res://autonomy/scenarios/scripted_walk.gd")

var def := {}
var ego: VehicleBody3D
var city: Node3D
var network: TrafficNetwork
var metrics: Metrics
var time := 0.0
var done := false
var result := {}
var seed_value := 0

var _jaywalker: Node3D
var _jaywalk_trigger_x := INF


func start(scenario: Dictionary, ego_car: VehicleBody3D, city_root: Node3D, net: TrafficNetwork, run_seed: int) -> void:
	def = scenario
	ego = ego_car
	city = city_root
	network = net
	seed_value = run_seed
	metrics = Metrics.new()
	metrics.name = "Metrics"
	add_child(metrics)
	metrics.attach(ego, network)
	var goal_lane := city.get_node(def.goal[0]) as RoadLane
	if not ego.autopilot.set_goal(goal_lane, def.goal[1]):
		push_error("Scenario %s: no route to the goal" % def.name)
	ego.autonomous = true
	match def.name:
		"red_light":
			_setup_red_light()
		"jaywalker":
			_setup_jaywalker()
		"roundabout":
			_spawn_npcs([["RoadManager/Ave_E2/RP_001/pF0_nF0", 30.0], ["RoadManager/Cross_N3/RP_001/pF0_nF0", 30.0],
					["RoadManager/Ave_E1/RP_001/pR0_nR0", 22.0], ["RoadManager/Ave_E2/RP_001/pF0_nF0", 5.0]])
		"highway_merge":
			var npcs := []
			# Timed to reach the merge point about when the ego car does.
			for off in [20.0, 50.0, 80.0, 110.0, 140.0, 170.0]:
				npcs.append(["RoadManager/HW_Loop/RP_007/pR2_nR2", off])
			_spawn_npcs(npcs)


func _physics_process(delta: float) -> void:
	if done or def.is_empty():
		return
	time += delta
	if Session.trace and fmod(time, 1.0) < delta:
		var ap: Autopilot = ego.autopilot
		print("t=%5.1f pos=%s v=%5.2f tgt=%5.2f a=%+.2f st=%s (%s) lane=%s off=%.1f err=%.2f objs=%d" % [time,
				ego.global_position.snapped(Vector3.ONE * 0.1), ego.speed(), ap.behaviour.target_speed, ap.accel,
				ap.behaviour.state_name(), ap.behaviour.reason,
				ap.loc.lane().name if ap.loc.has_route() else "-", ap.loc.offset, ap.loc.lane_error, ap.perception.objects.size()])
		if ap.path:
			print("   events: ", ap.path.events.map(func(e): return "%s@%.0f" % [e.kind, e.s]), " route: ", ap.loc.route.slice(ap.loc.idx, ap.loc.idx + 4).map(func(r): return "%s/%s:%s" % [r.lane.get_parent().name, r.lane.name, r.kind]))
		if def.name == "highway_merge":
			var near := []
			for c in city.get_node("RoadManager/vehicles").get_children():
				if c.global_position.distance_to(ego.global_position) < 120.0:
					near.append(str(c.global_position.round()) + "@" + str(snappedf(c.speed, 0.1)))
			print("   npcs<120m: ", near)
	if _jaywalker and ego.global_position.x >= _jaywalk_trigger_x:
		_jaywalk_trigger_x = INF
		_jaywalker._wait = 0.0 # step off the kerb now
	if ego.autopilot.has_arrived:
		_finish("arrived")
	elif time >= def.time_limit:
		_finish("timeout")


func _finish(how: String) -> void:
	done = true
	var m := metrics.to_dict()
	var reached := how == "arrived"
	var failures := []
	if not reached:
		failures.append("did not reach goal")
	if m.collisions > 0:
		failures.append("%d collision(s)" % m.collisions)
	if m.red_light_violations > 0:
		failures.append("%d red-light violation(s)" % m.red_light_violations)
	if m.min_ped_distance >= 0.0 and m.min_ped_distance < Scenarios.MIN_PED_DISTANCE:
		failures.append("pedestrian too close (%.2f m)" % m.min_ped_distance)
	if m.lane_dev_max > Scenarios.MAX_LANE_DEVIATION:
		failures.append("lane deviation %.2f m" % m.lane_dev_max)
	result = {
		scenario = def.name,
		seed = seed_value,
		passed = failures.is_empty(),
		failures = failures,
		reached_goal = reached,
		time = snappedf(time, 0.01),
		route_replans = ego.autopilot.replans,
	}
	result.merge(m)
	finished.emit(result)


func _setup_red_light() -> void:
	# The light ahead is green with 2.5 s left when the car sets off.
	var lane := city.get_node(def.start[0]) as RoadLane
	var controller: TrafficLightController = lane.get_meta("signal")
	var group: int = controller.arm_group[lane.get_meta("arm")]
	for i in controller._phases.size():
		if controller._phases[i].get("green", -1) == group:
			controller._phase = i
			controller._time_left = 2.5
			controller._apply()
			break


func _setup_jaywalker() -> void:
	# Mid-block on Cross_S1, stepping off the right-hand kerb.
	var x := -88.0
	var walk := ScriptedWalk.new()
	walk.name = "JaywalkPath"
	walk.setup(Vector3(x, 0.15, 60.2), Vector3(x, 0.15, 37.8))
	add_child(walk)
	_jaywalker = PEDESTRIAN.instantiate()
	_jaywalker.network = walk
	_jaywalker.ring = 0
	_jaywalker.s = 0.0
	_jaywalker.state = _jaywalker.State.WAIT
	_jaywalker._link = walk.crossing()
	_jaywalker._wait = INF
	add_child(_jaywalker)
	_jaywalker._speed = 1.5
	_jaywalk_trigger_x = x - 28.0


func _spawn_npcs(spots: Array) -> void:
	var vehicles := city.get_node("RoadManager/vehicles")
	for spot in spots:
		var lane := city.get_node(spot[0]) as RoadLane
		var pos: Vector3 = network.sample(lane, spot[1])[0]
		var car = vehicles.add_actor(pos, lane)
		if car:
			car.place(network, lane, spot[1])
	if Session.trace:
		print("spawned %d scenario NPCs" % spots.size())
