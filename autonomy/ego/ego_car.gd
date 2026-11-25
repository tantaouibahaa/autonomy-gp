extends VehicleBody3D

## The ego car: a VehicleBody3D driven either by the keyboard or by the
## Autopilot child node. Press T to switch between the two.
##
## The autopilot asks for an acceleration in m/s^2; this script maps it to
## engine force or brake using the car's measured response (about 9 m/s^2 at
## full engine force, about 0.28 m/s^2 per unit of brake).

signal mode_changed(autonomous: bool)
signal collided(body: Node)

@export var max_engine_force := 6000.0
@export var max_brake := 40.0
@export var max_steer := 0.6
@export var steer_rate := 2.0
## Engine force per m/s^2 of acceleration and deceleration per unit of brake.
@export var force_per_accel := 650.0
@export var decel_per_brake := 0.28
@export var autonomous := true: set = set_autonomous

@onready var autopilot: Autopilot = $Autopilot
@onready var perception: Perception = $Perception
@onready var front_lidar: Lidar = $FrontLidar
@onready var surround_lidar: Lidar = $SurroundLidar


func _ready() -> void:
	body_entered.connect(func(b): collided.emit(b))


func set_autonomous(value: bool) -> void:
	autonomous = value
	mode_changed.emit(value)


func speed() -> float:
	return linear_velocity.dot(global_basis.z)


func _physics_process(delta: float) -> void:
	autopilot.step(delta)
	if autonomous:
		var a := autopilot.accel
		if a >= 0.0:
			engine_force = minf(a * force_per_accel, max_engine_force)
			brake = 0.0
		else:
			engine_force = 0.0
			brake = minf(-a / decel_per_brake, max_brake)
		steering = move_toward(steering, autopilot.steer, steer_rate * delta)
	else:
		var throttle := Input.get_axis("ui_down", "ui_up")
		if Input.is_key_pressed(KEY_W):
			throttle = 1.0
		elif Input.is_key_pressed(KEY_S):
			throttle = -1.0
		var turn := Input.get_axis("ui_right", "ui_left")
		if Input.is_key_pressed(KEY_A):
			turn = 1.0
		elif Input.is_key_pressed(KEY_D):
			turn = -1.0
		# Ease off the steering at speed so the car stays planted.
		var limit := max_steer / (1.0 + absf(speed()) / 12.0)
		steering = move_toward(steering, turn * limit, steer_rate * delta)
		engine_force = throttle * max_engine_force * 0.6
		brake = max_brake * 0.7 if Input.is_key_pressed(KEY_SPACE) else 0.0
