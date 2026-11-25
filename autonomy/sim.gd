extends Node3D

## Entry scene: the city from node_3d.tscn plus the autonomous ego car, its
## camera and HUD, and whatever Session says to run (free drive, a scenario,
## or the time-trial race).
##
## Keys: T autopilot on/off, 1-6 scenarios, 0 free drive, R race,
## G new random destination, L lidar view, C camera, Esc quit.

const EGO_SCENE := preload("res://autonomy/ego/ego_car.tscn")
const CITY_SCENE := preload("res://node_3d.tscn")
const FREE_START := ["RoadManager/Cross_S1/RP_001/pR0_nR0", 10.0]
const RACE_START := ["RoadManager/HW_Onramp/HW_01/pF1_nF1", 4.0]

var city: Node3D
@onready var hud: Hud = $Hud
@onready var cam: ChaseCamera = $ChaseCamera

var network: TrafficNetwork
var ego: VehicleBody3D
var director: ScenarioDirector
var race: RaceMode
var _seed := 0


func _enter_tree() -> void:
	_seed = Session.config.seed
	if _seed == 0:
		_seed = randi() % 100000 + 1
	seed(_seed)
	if Session.batch and Session.config.mode == "free":
		return # the batch parent process only relaunches itself
	city = CITY_SCENE.instantiate()
	city.name = "City"
	add_child(city)
	move_child(city, 0)


func _ready() -> void:
	if Session.batch and Session.config.mode == "free":
		set_physics_process(false)
		return # the batch parent process only relaunches itself
	var stray := city.get_node_or_null("RoadManager/PlayerCar")
	if stray:
		stray.free() # replaced by the ego car below
	network = city.get_node("TrafficNetwork")

	var mode: String = Session.config.mode
	var start: Array = FREE_START
	if mode == "scenario":
		start = Scenarios.get_def(Session.config.scenario).start
	elif mode == "race":
		start = RACE_START
	ego = EGO_SCENE.instantiate()
	add_child(ego)
	_place(ego, city.get_node(start[0]), start[1])
	ego.autopilot.setup(network)

	var seeder := city.get_node("TrafficSeeder")
	seeder.rng_seed = _seed
	seeder.keep_clear_of = seeder.get_path_to(ego)

	cam.target = ego
	hud.ego = ego
	hud.seed_value = _seed
	if Session.batch:
		ego.front_lidar.show_debug = false

	if not network.is_built:
		await network.built
	await get_tree().physics_frame # let the seeder place its cars first

	match mode:
		"scenario":
			director = ScenarioDirector.new()
			director.name = "ScenarioDirector"
			add_child(director)
			director.finished.connect(_on_scenario_finished)
			director.start(Scenarios.get_def(Session.config.scenario), ego, city, network, _seed)
			hud.director = director
		"race":
			race = RaceMode.new()
			race.name = "RaceMode"
			add_child(race)
			race.start(ego, city, network, Session.config.laps)
			race.finished.connect(Session.report_race)
			hud.race = race
		_:
			ego.autopilot.arrived.connect(_random_goal)
			_random_goal()


func _place(car: VehicleBody3D, lane: RoadLane, offset: float) -> void:
	var sp: Array = network.sample(lane, offset)
	var dir: Vector3 = sp[1]
	dir.y = 0
	car.global_transform = Transform3D(Basis.looking_at(-dir.normalized(), Vector3.UP), sp[0] + Vector3.UP * 0.35)


func _random_goal() -> void:
	var options := network.lanes.filter(func(l): return not l.has_meta("turn") \
			and network.lane_length(l) > 40.0 and network._junction_of(l) == null)
	for _i in 20:
		var lane: RoadLane = options.pick_random()
		var off := randf_range(10.0, network.lane_length(lane) - 25.0)
		if network.sample(lane, off)[0].distance_to(ego.global_position) > 150.0 \
				and ego.autopilot.set_goal(lane, off):
			hud.flash("New destination: %s" % _lane_label(lane))
			return


func _lane_label(lane: RoadLane) -> String:
	return str(city.get_path_to(lane)).replace("RoadManager/", "")


func _on_scenario_finished(result: Dictionary) -> void:
	hud.show_result(result)
	if Session.batch:
		Session.report(result)
	else:
		print(JSON.stringify(result))


func _unhandled_input(event: InputEvent) -> void:
	if not event is InputEventKey or not event.pressed or event.echo:
		return
	var key: int = event.keycode
	if key >= KEY_1 and key <= KEY_6:
		var names := Scenarios.names()
		Session.start("scenario", names[key - KEY_1], 0)
	match key:
		KEY_0:
			Session.start("free")
		KEY_R:
			Session.start("race")
		KEY_T:
			ego.autonomous = not ego.autonomous
			hud.flash("Autopilot ON" if ego.autonomous else "Manual driving")
		KEY_G:
			_random_goal()
		KEY_L:
			ego.front_lidar.show_debug = not ego.front_lidar.show_debug
			ego.surround_lidar.show_debug = ego.front_lidar.show_debug
		KEY_C:
			cam.cycle()
		KEY_ESCAPE:
			get_tree().quit()
