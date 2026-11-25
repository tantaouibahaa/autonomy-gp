extends AnimatableBody3D

## A pedestrian walking around the sidewalk rings of pedestrian_spawner.gd.
## Uses crosswalks only while the junction shows WALK. Jaywalkers also cross
## mid-block now and then, without looking.

enum State { WALK, WAIT, CROSS }

## Character models to pick from at random. Kenney models face +Z.
@export var models: Array[PackedScene] = []
@export var model_scale := 0.65
@export var speed_range := Vector2(1.1, 1.6)
## Height of the sidewalk top and of the road surface.
@export var sidewalk_height := 0.15
@export var road_height := 0.02

var jaywalker := false
var network: Node
var ring := 0
var s := 0.0
var walk_dir := 1
var state := State.WALK

var _speed := 1.3
var _link := {}
var _cross_t := 0.0
var _wait := 0.0
var _anim: AnimationPlayer


func _ready() -> void:
	add_to_group("pedestrians")
	_speed = randf_range(speed_range.x, speed_range.y)
	if not models.is_empty():
		var model: Node3D = models.pick_random().instantiate()
		model.scale = Vector3.ONE * model_scale
		add_child(model)
		_anim = model.find_child("AnimationPlayer", true, false)
		for anim_name in ["walk", "idle"]:
			if _anim and _anim.has_animation(anim_name):
				_anim.get_animation(anim_name).loop_mode = Animation.LOOP_LINEAR
	global_position = network.ring_point(ring, s)
	_play("walk")


func _physics_process(delta: float) -> void:
	var before := global_position
	match state:
		State.WALK:
			_walk(delta)
		State.WAIT:
			_play("idle")
			if _link.kind == "crosswalk":
				if _link.controller.is_walk():
					_start_cross()
			else:
				_wait -= delta
				if _wait <= 0.0:
					_start_cross()
		State.CROSS:
			_cross(delta)
	var moved := global_position - before
	moved.y = 0
	if moved.length() > 0.001:
		global_basis = Basis.looking_at(-moved.normalized(), Vector3.UP)


func _walk(delta: float) -> void:
	_play("walk")
	var step := _speed * delta
	var length: float = network.ring_length[ring]
	for link in network.links[ring]:
		var ahead := fposmod((link.s - s) * walk_dir, length)
		if ahead > 0.0 and ahead <= step and _wants(link):
			s = link.s
			_link = link
			_wait = randf_range(0.5, 3.0)
			state = State.WAIT
			global_position = network.ring_point(ring, s)
			return
	s = fposmod(s + step * walk_dir, length)
	global_position = network.ring_point(ring, s)


func _wants(link: Dictionary) -> bool:
	if link.kind == "crosswalk":
		return randf() < network.cross_chance
	return jaywalker and randf() < network.jaywalk_chance


func _start_cross() -> void:
	_cross_t = 0.0
	state = State.CROSS
	_play("walk")


func _cross(delta: float) -> void:
	var a: Vector3 = _link.a
	var b: Vector3 = _link.b
	var dist := a.distance_to(b)
	_cross_t = minf(_cross_t + _speed * delta / dist, 1.0)
	var pos := a.lerp(b, _cross_t)
	var from_curb := minf(_cross_t, 1.0 - _cross_t) * dist
	pos.y = sidewalk_height if from_curb < 1.6 else road_height
	global_position = pos
	if _cross_t >= 1.0:
		ring = _link.to_ring
		s = _link.to_s
		walk_dir = 1 if randf() < 0.5 else -1
		state = State.WALK


func _play(anim_name: String) -> void:
	if _anim and _anim.has_animation(anim_name) and _anim.current_animation != anim_name:
		_anim.play(anim_name)
