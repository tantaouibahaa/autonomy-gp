class_name Metrics
extends Node

## Ground-truth scoring of a drive. Unlike the autopilot, this reads the
## simulator directly: actual signal states, actual pedestrian positions.

var collisions := 0
var collision_log: Array = []
var red_light_violations := 0
var min_ped_distance := INF
var lane_dev_max := 0.0
var lane_dev_mean := 0.0
var jerk_rms := 0.0
var jerk_max := 0.0
var max_decel := 0.0
var distance := 0.0
var time := 0.0
var states := {} # state name -> seconds

var _ego: VehicleBody3D
var _network: TrafficNetwork
var _dev_sum := 0.0
var _dev_n := 0
var _jerk_sq := 0.0
var _jerk_n := 0
var _v_prev := NAN
var _a_filt := 0.0
var _a_prev := NAN
var _line_prev := {} # lane -> front offset last frame
var _last_hit := {} # body id -> time
var _half_length := 2.0
var _half_width := 0.95


func attach(ego: VehicleBody3D, network: TrafficNetwork) -> void:
	_ego = ego
	_network = network
	ego.collided.connect(_on_collided)


func _on_collided(body: Node) -> void:
	if not body is CollisionObject3D:
		return
	if (body.collision_layer & (2 | 4 | 8 | 16)) == 0:
		return # road, ground, kerbs
	var id := body.get_instance_id()
	if time - _last_hit.get(id, -INF) > 2.0:
		collisions += 1
		var rel: Vector3 = _ego.global_transform.affine_inverse() * (body as Node3D).global_position
		collision_log.append({time = snappedf(time, 0.01), what = String(body.get_parent().name if body.name == "Body" else body.name),
				cls = Lidar.classify(body), speed = snappedf(_ego.speed(), 0.01),
				# where the other body was, in the car's frame (+z ahead, +x left)
				where = [snappedf(rel.x, 0.1), snappedf(rel.z, 0.1)]})
	_last_hit[id] = time


func _physics_process(delta: float) -> void:
	if _ego == null:
		return
	time += delta
	var v: float = _ego.speed()
	distance += absf(v) * delta
	if not is_nan(_v_prev):
		var a := (v - _v_prev) / delta
		# Low-pass the raw acceleration (contact jitter), then differentiate.
		_a_filt = lerpf(_a_filt, a, clampf(delta / 0.25, 0.0, 1.0))
		max_decel = maxf(max_decel, -_a_filt)
		# Jerk only while rolling: at a standstill the tyre contacts jitter.
		if not is_nan(_a_prev) and absf(v) > 0.5:
			var j := (_a_filt - _a_prev) / delta
			jerk_max = maxf(jerk_max, absf(j))
			_jerk_sq += j * j
			_jerk_n += 1
			jerk_rms = sqrt(_jerk_sq / _jerk_n)
		_a_prev = _a_filt
	_v_prev = v

	var ap: Autopilot = _ego.autopilot
	if ap.loc and ap.loc.has_route():
		var dev := ap.loc.lane_error
		lane_dev_max = maxf(lane_dev_max, dev)
		_dev_sum += dev
		_dev_n += 1
		lane_dev_mean = _dev_sum / _dev_n
		_check_signal(ap.loc.lane())
		var st := ap.behaviour.state_name()
		states[st] = states.get(st, 0.0) + delta
	_check_pedestrians()


## A violation is the front bumper crossing a stop line while it shows red.
func _check_signal(lane: RoadLane) -> void:
	if lane == null or not lane.has_meta("signal"):
		return
	var fwd := _ego.global_basis.z
	var front := _ego.global_position + fwd * _half_length
	var off := lane.curve.get_closest_offset(lane.to_local(front))
	var line: float = lane.get_meta("stop_offset") + _network.stop_margin
	var prev: float = _line_prev.get(lane, off)
	_line_prev[lane] = off
	if prev < line and off >= line:
		var controller: TrafficLightController = lane.get_meta("signal")
		if controller.get_state(lane.get_meta("arm")) == TrafficLightController.Light.RED:
			red_light_violations += 1


func _check_pedestrians() -> void:
	var xf := _ego.global_transform
	for ped in get_tree().get_nodes_in_group("pedestrians"):
		var p: Vector3 = ped.global_position
		if p.distance_squared_to(xf.origin) > 400.0:
			continue
		var local := xf.affine_inverse() * p
		var dx := maxf(absf(local.x) - _half_width, 0.0)
		var dz := maxf(absf(local.z) - _half_length, 0.0)
		var d := Vector2(dx, dz).length() - 0.3
		min_ped_distance = minf(min_ped_distance, maxf(d, 0.0))


func to_dict() -> Dictionary:
	var st := {}
	for k in states:
		st[k] = snappedf(states[k], 0.1)
	return {
		collisions = collisions,
		collision_log = collision_log,
		red_light_violations = red_light_violations,
		min_ped_distance = snappedf(min_ped_distance, 0.01) if min_ped_distance < INF else -1.0,
		lane_dev_mean = snappedf(lane_dev_mean, 0.001),
		lane_dev_max = snappedf(lane_dev_max, 0.001),
		jerk_rms = snappedf(jerk_rms, 0.01),
		jerk_max = snappedf(jerk_max, 0.01),
		max_decel = snappedf(max_decel, 0.01),
		distance = snappedf(distance, 0.1),
		state_time = st,
	}
