class_name PID
extends RefCounted

## Textbook PID with integral clamping (anti-windup).

var kp: float
var ki: float
var kd: float
var i_limit: float

var _i := 0.0
var _prev := 0.0
var _has_prev := false


func _init(p: float, i: float, d: float, limit := 2.0) -> void:
	kp = p
	ki = i
	kd = d
	i_limit = limit


func step(error: float, dt: float) -> float:
	_i = clampf(_i + error * dt, -i_limit, i_limit)
	var deriv := (error - _prev) / dt if _has_prev and dt > 0.0 else 0.0
	_prev = error
	_has_prev = true
	return kp * error + ki * _i + kd * deriv


func reset() -> void:
	_i = 0.0
	_has_prev = false
