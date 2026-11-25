class_name Hud
extends CanvasLayer

## On-screen readout of the stack: mode, behaviour state, speed vs limit,
## what the traffic-light camera sees, lane, and the scenario or race score.

const LIGHT_NAMES := {-1: "unseen", 0: "RED", 1: "YELLOW", 2: "GREEN"}
const LIGHT_COLORS := {-1: Color(0.7, 0.7, 0.7), 0: Color(1, 0.3, 0.25), 1: Color(1, 0.8, 0.2), 2: Color(0.3, 1, 0.45)}

var ego: VehicleBody3D
var director: ScenarioDirector
var race: RaceMode
var seed_value := 0

var _left: Label
var _right: Label
var _state: Label
var _light: Label
var _flash: Label
var _help: Label
var _flash_time := 0.0
var _result := ""


func _ready() -> void:
	_left = _label(Vector2(16, 14), 17)
	_state = _label(Vector2(16, 0), 26)
	_state.anchor_top = 0
	_state.position = Vector2(16, 150)
	_light = _label(Vector2(16, 186), 20)
	_right = _label(Vector2(0, 14), 17)
	_right.anchor_left = 1.0
	_right.anchor_right = 1.0
	_right.offset_left = -360
	_right.offset_right = -16
	_right.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	_flash = _label(Vector2(0, 0), 30)
	_flash.anchor_left = 0.5
	_flash.anchor_right = 0.5
	_flash.anchor_top = 0.25
	_flash.offset_left = -400
	_flash.offset_right = 400
	_flash.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_help = _label(Vector2(16, 0), 14)
	_help.anchor_top = 1.0
	_help.anchor_bottom = 1.0
	_help.offset_top = -30
	_help.modulate = Color(1, 1, 1, 0.75)
	_help.text = "T autopilot/manual   WASD/arrows drive   1-6 scenarios   0 free drive   R race   G new destination   L lidar   C camera   Esc quit"


func _label(pos: Vector2, size: int) -> Label:
	var l := Label.new()
	l.position = pos
	l.add_theme_font_size_override("font_size", size)
	l.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.85))
	l.add_theme_constant_override("outline_size", 6)
	add_child(l)
	return l


func flash(text: String) -> void:
	_flash.text = text
	_flash_time = 2.5


func show_result(r: Dictionary) -> void:
	_result = "%s: %s" % [r.scenario.to_upper(), "PASS" if r.passed else "FAIL - " + ", ".join(r.failures)]
	flash(_result)
	_flash_time = 1e9


func _process(delta: float) -> void:
	_flash_time -= delta
	_flash.visible = _flash_time > 0.0
	if ego == null or not is_instance_valid(ego):
		return
	var ap: Autopilot = ego.autopilot
	var b := ap.behaviour
	var v: float = ego.speed()
	var limit := 0.0
	var lane_name := "-"
	if ap.loc and ap.loc.has_route():
		limit = ap.network.speed_limit(ap.loc.lane())
		lane_name = str(ap.loc.lane().get_parent().get_parent().name) + "/" + str(ap.loc.lane().name)
	var lines := PackedStringArray()
	lines.append("%s   %3d km/h   limit %d" % ["AUTOPILOT" if ego.autonomous else "MANUAL", roundi(v * 3.6), roundi(limit * 3.6)])
	lines.append("target %3d km/h   accel %+.1f m/s2   steer %+.0f deg" % [roundi(b.target_speed * 3.6), ap.accel, rad_to_deg(ego.steering)])
	lines.append("lane %s   lateral err %.2f m" % [lane_name, ap.loc.lane_error if ap.loc else 0.0])
	lines.append("tracked objects %d   lidar returns %d" % [ap.perception.objects.size(), ap.perception.points.size()])
	if ap.path:
		var goal_s := -1.0
		for ev in ap.path.events:
			if ev.kind == "goal":
				goal_s = ev.s
		lines.append("goal %s" % ("%.0f m" % goal_s if goal_s >= 0.0 else "> %.0f m" % ap.path.length))
	_left.text = "\n".join(lines)
	_state.text = "%s  (%s)" % [b.state_name(), b.reason]
	_state.modulate = Color(1, 0.35, 0.3) if b.state == Behaviour.State.EMERGENCY_BRAKE else Color(1, 1, 1)
	if b.light.is_empty():
		_light.text = ""
	else:
		var st: int = b.light.state
		_light.text = "signal %s  %.0f m%s" % [LIGHT_NAMES.get(st, "?"), b.light.distance, "" if b.light.visible else " (memory)"]
		_light.modulate = LIGHT_COLORS.get(st, Color.WHITE)

	var r := PackedStringArray()
	if director:
		var m := director.metrics
		r.append("Scenario: %s   seed %d" % [director.def.title, seed_value])
		r.append("time %.1f / %.0f s" % [director.time, director.def.time_limit])
		r.append("collisions %d   red-light violations %d" % [m.collisions, m.red_light_violations])
		r.append("min pedestrian distance %s" % ("%.2f m" % m.min_ped_distance if m.min_ped_distance < INF else "-"))
		r.append("lane deviation %.2f / %.2f m" % [m.lane_dev_mean, m.lane_dev_max])
		r.append("jerk rms %.1f m/s3" % m.jerk_rms)
	elif race:
		r.append("TIME TRIAL - HW_Loop")
		r.append("lap %d / %d" % [mini(maxi(race.lap, 1), race.laps_target), race.laps_target])
		r.append("current %s" % _clock(race.current_lap_time()))
		r.append("best %s" % (_clock(race.best_lap) if race.best_lap < INF else "--:--.--"))
		for i in race.lap_times.size():
			r.append("lap %d  %s" % [i + 1, _clock(race.lap_times[i])])
	else:
		r.append("Free drive   seed %d" % seed_value)
	_right.text = "\n".join(r)


func _clock(t: float) -> String:
	return "%d:%05.2f" % [int(t) / 60, fmod(t, 60.0)]
