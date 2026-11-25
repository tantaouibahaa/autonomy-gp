extends Node

## Autoload that survives scene reloads: what the next run of sim.tscn should
## do (free drive, a scenario, the race) and, in batch mode, the queue of
## scenario runs and their results.
##
## Batch mode, from the project folder:
##   godot --headless --path . -- --scenarios all [--seeds 1,2,3] [--out results]
##   godot --headless --path . -- --race [--laps 2]
## The runner restarts itself once (child gets --no-relaunch) with
## `--fixed-fps 60`, so the simulation runs as fast as the CPU allows while the
## physics still steps at exactly 60 Hz.

const SIM := "res://autonomy/sim.tscn"

## {mode: "free"/"scenario"/"race", scenario, seed, laps}
var config := {mode = "free", scenario = "", seed = 0, laps = 3}
var batch := false
## --trace: print the car's state once a second during scenario runs.
var trace := false
var queue: Array = []
var results: Array = []
var out_dir := "res://results"
var race_result := {}

var _child_pid := -1
var _run_time := 0.0
var _reported := true


## Batch watchdog: a run that never reports (script error, stuck scene) is
## recorded as a failure instead of hanging the batch.
func _physics_process(delta: float) -> void:
	if not batch or _reported or config.mode != "scenario":
		return
	_run_time += delta
	var limit: float = Scenarios.get_def(config.scenario).time_limit + 30.0
	if _run_time > limit:
		report({scenario = config.scenario, seed = config.seed, passed = false,
				failures = ["watchdog: run never finished"], reached_goal = false, time = _run_time,
				collisions = -1, red_light_violations = -1, min_ped_distance = -1.0,
				lane_dev_mean = -1.0, lane_dev_max = -1.0, jerk_rms = -1.0, jerk_max = -1.0,
				max_decel = -1.0, distance = -1.0})


func _ready() -> void:
	var args := OS.get_cmdline_user_args()
	var scen := _arg(args, "--scenarios")
	var race := "--race" in args
	if scen == "" and not race:
		return
	batch = true
	trace = "--trace" in args
	out_dir = _arg(args, "--out", "res://results")
	if not "--no-relaunch" in args and OS.get_environment("AUTONOMY_GP_CHILD") == "":
		_relaunch(args)
		return
	var seeds := PackedInt32Array()
	for x in _arg(args, "--seeds", "1,2,3").split(","):
		seeds.append(int(x))
	if race:
		var laps := int(_arg(args, "--laps", "1"))
		queue.append({mode = "race", scenario = "", seed = seeds[0], laps = laps})
	else:
		var names := Scenarios.names() if scen == "all" else scen.split(",")
		for n in names:
			if Scenarios.get_def(n).is_empty():
				printerr("Unknown scenario: ", n, " (have: ", ", ".join(Scenarios.names()), ")")
				get_tree().quit(2)
				return
			for s in seeds:
				queue.append({mode = "scenario", scenario = n, seed = s, laps = 0})
	print("autonomy-gp batch: %d run(s)" % queue.size())
	_next.call_deferred()


## Start a run interactively (from the in-game keys).
func start(mode: String, scenario := "", run_seed := 0) -> void:
	config = {mode = mode, scenario = scenario, seed = run_seed if run_seed != 0 else randi() % 100000 + 1, laps = 3}
	get_tree().change_scene_to_file(SIM)


## Called by sim.gd when a scenario run ends.
func report(result: Dictionary) -> void:
	if not batch or _reported:
		return
	_reported = true
	results.append(result)
	var line := "%-14s seed %-3d %s  t=%6.1fs  coll=%d  red=%d  ped=%5.2fm  dev=%.2f/%.2fm  jerk=%.1f" % [
			result.scenario, result.seed, "PASS" if result.passed else "FAIL", result.time,
			result.collisions, result.red_light_violations, result.min_ped_distance,
			result.lane_dev_mean, result.lane_dev_max, result.jerk_rms]
	if not result.passed:
		line += "  <- " + ", ".join(result.failures)
	print(line)
	_next.call_deferred()


func report_race(result: Dictionary) -> void:
	race_result = result
	print("race: ", JSON.stringify(result))
	if batch:
		_write_race()
		get_tree().quit(0)


func _next() -> void:
	if queue.is_empty():
		_write_results()
		var failed := results.filter(func(r): return not r.passed).size()
		print("%d/%d passed" % [results.size() - failed, results.size()])
		get_tree().quit(0 if failed == 0 else 1)
		return
	config = queue.pop_front()
	_run_time = 0.0
	_reported = false
	get_tree().change_scene_to_file(SIM)


func _write_results() -> void:
	DirAccess.make_dir_recursive_absolute(out_dir)
	var summary := {}
	for r in results:
		var s: Dictionary = summary.get_or_add(r.scenario, {runs = 0, passed = 0, collisions = 0,
				red_light_violations = 0, min_ped_distance = INF, lane_dev_max = 0.0, time = 0.0, jerk_rms = 0.0})
		s.runs += 1
		s.passed += 1 if r.passed else 0
		s.collisions += r.collisions
		s.red_light_violations += r.red_light_violations
		if r.min_ped_distance >= 0.0:
			s.min_ped_distance = minf(s.min_ped_distance, r.min_ped_distance)
		s.lane_dev_max = maxf(s.lane_dev_max, r.lane_dev_max)
		s.time += r.time / 1.0
		s.jerk_rms += r.jerk_rms
	for k in summary:
		var s: Dictionary = summary[k]
		s.time = snappedf(s.time / s.runs, 0.1)
		s.jerk_rms = snappedf(s.jerk_rms / s.runs, 0.01)
		if s.min_ped_distance == INF:
			s.min_ped_distance = -1.0
	var doc := {
		generated_by = "godot --headless --path . -- --scenarios all",
		engine = Engine.get_version_info().string,
		physics_hz = Engine.physics_ticks_per_second,
		criteria = {reach_goal = true, collisions = 0, red_light_violations = 0,
				min_ped_distance_m = Scenarios.MIN_PED_DISTANCE, max_lane_deviation_m = Scenarios.MAX_LANE_DEVIATION},
		summary = summary,
		runs = results,
	}
	var f := FileAccess.open(out_dir.path_join("scenario_results.json"), FileAccess.WRITE)
	f.store_string(JSON.stringify(doc, "  ", false))
	f.close()
	var cols := ["scenario", "seed", "passed", "time", "reached_goal", "collisions", "red_light_violations",
			"min_ped_distance", "lane_dev_mean", "lane_dev_max", "jerk_rms", "jerk_max", "max_decel", "distance"]
	var csv := FileAccess.open(out_dir.path_join("scenario_results.csv"), FileAccess.WRITE)
	csv.store_line(",".join(cols))
	for r in results:
		csv.store_line(",".join(cols.map(func(c): return str(r[c]))))
	csv.close()
	print("results written to ", ProjectSettings.globalize_path(out_dir))


func _write_race() -> void:
	DirAccess.make_dir_recursive_absolute(out_dir)
	var f := FileAccess.open(out_dir.path_join("race_result.json"), FileAccess.WRITE)
	f.store_string(JSON.stringify(race_result, "  ", false))
	f.close()


## Re-run this same command with --fixed-fps 60, streaming its output.
func _relaunch(args: PackedStringArray) -> void:
	var cmd := PackedStringArray(["--headless", "--path", ProjectSettings.globalize_path("res://"),
			"--fixed-fps", "60", "--", "--no-relaunch"])
	cmd.append_array(args)
	OS.set_environment("AUTONOMY_GP_CHILD", "1")
	var proc := OS.execute_with_pipe(OS.get_executable_path(), cmd, false)
	if proc.is_empty():
		printerr("could not relaunch with --fixed-fps; running in real time")
		return
	_child_pid = proc.pid
	var io: FileAccess = proc.stdio
	while OS.is_process_running(_child_pid):
		var line := io.get_line()
		if line != "":
			print(line)
		else:
			OS.delay_msec(20)
	while true:
		var rest := io.get_line()
		if rest == "" and io.eof_reached():
			break
		if rest != "":
			print(rest)
	get_tree().quit(OS.get_process_exit_code(_child_pid))


func _arg(args: PackedStringArray, key: String, default := "") -> String:
	var i := args.find(key)
	if i < 0:
		return default
	if i + 1 < args.size() and not args[i + 1].begins_with("--"):
		return args[i + 1]
	return "all" if key == "--scenarios" else default
