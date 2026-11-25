class_name RaceMode
extends Node

## "GP" mode: a time trial round the HW_Loop highway circuit, in live traffic.
## The autopilot drives at up to 120% of the posted limit and overtakes slow
## cars; the clock starts on the first pass of the start/finish line.

signal finished(result: Dictionary)

## Waypoints that keep the route on the circuit (A* picks the lanes between).
const WAYPOINTS := [
	["RoadManager/HW_Loop/RP_004/pR1_nR1", 450.0],
	["RoadManager/HW_Loop/RP_007/pR1_nR1", 250.0],
	["RoadManager/HW_Middle/RP_001/pR1_nR1", 100.0],
	["RoadManager/HW_Loop/RP_001/pR1_nR1", 150.0],
]
## Start/finish line: where HW_Loop begins (x = 176), across its three lanes.
const LINE_X := 176.0
const LINE_Z := Vector2(-347.0, -333.0)

var ego: VehicleBody3D
var city: Node3D
var network: TrafficNetwork
var metrics: Metrics
var laps_target := 3
var lap := 0 # laps started
var lap_start := 0.0
var lap_times: Array[float] = []
var best_lap := INF
var time := 0.0
var done := false

var _wp := 0
var _prev_x := 0.0


func start(car: VehicleBody3D, city_root: Node3D, net: TrafficNetwork, laps: int) -> void:
	ego = car
	city = city_root
	network = net
	laps_target = maxi(laps, 1)
	metrics = Metrics.new()
	add_child(metrics)
	metrics.attach(ego, network)
	var ap: Autopilot = ego.autopilot
	ap.behaviour.limit_factor = 1.2
	ap.behaviour.overtake = true
	ap.lateral_accel = 4.0
	ego.autonomous = true
	_prev_x = ego.global_position.x
	_set_waypoint(0)


func current_lap_time() -> float:
	return time - lap_start if lap > 0 else 0.0


func _set_waypoint(i: int) -> void:
	_wp = i % WAYPOINTS.size()
	var w: Array = WAYPOINTS[_wp]
	ego.autopilot.set_goal(city.get_node(w[0]), w[1])


func _physics_process(delta: float) -> void:
	if ego == null or done:
		return
	time += delta
	var w: Array = WAYPOINTS[_wp]
	var goal_pos: Vector3 = network.sample(city.get_node(w[0]), w[1])[0]
	if ego.global_position.distance_to(goal_pos) < 70.0:
		_set_waypoint(_wp + 1)

	var p := ego.global_position
	if _prev_x < LINE_X and p.x >= LINE_X and p.z > LINE_Z.x and p.z < LINE_Z.y:
		if lap > 0:
			var t := time - lap_start
			lap_times.append(t)
			best_lap = minf(best_lap, t)
			print("lap %d: %.2f s" % [lap, t])
		if lap >= laps_target:
			_finish()
		else:
			lap += 1
			lap_start = time
	_prev_x = p.x


func _finish() -> void:
	done = true
	var m := metrics.to_dict()
	finished.emit({
		laps = lap_times.size(),
		lap_times = lap_times.map(func(t): return snappedf(t, 0.01)),
		best_lap = snappedf(best_lap, 0.01),
		average_speed_kmh = snappedf(m.distance / maxf(time, 0.1) * 3.6, 0.1),
		collisions = m.collisions,
		red_light_violations = m.red_light_violations,
		lane_dev_max = m.lane_dev_max,
	})
