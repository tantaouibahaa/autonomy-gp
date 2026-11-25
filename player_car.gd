extends VehicleBody3D

@export var max_engine_force := 3000.0
@export var max_brake := 50.0
@export var max_steer := 0.5 # radians, about 30 degrees

func _physics_process(delta: float) -> void:
	var steer_input := Input.get_axis("ui_right", "ui_left")
	# Less steering lock at speed, so hard turns don't roll the car over.
	var speed := absf(linear_velocity.dot(global_basis.z))
	var lock := max_steer / (1.0 + speed / 12.0)
	steering = move_toward(steering, steer_input * lock, delta * 2.0)
	engine_force = Input.get_axis("ui_down", "ui_up") * max_engine_force
	brake = max_brake if Input.is_key_pressed(KEY_SPACE) else 0.0
	  
