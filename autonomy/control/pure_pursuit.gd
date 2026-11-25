class_name PurePursuit
extends RefCounted

## Pure-pursuit steering: aim the rear axle at a point `lookahead` meters
## down the reference path and steer onto the circular arc that reaches it.
##   delta = atan(2 L sin(alpha) / Ld)

var wheelbase := 1.98
var gain := 0.45 # lookahead seconds
var min_lookahead := 5.0
var max_lookahead := 18.0

## Last target point, for debug drawing.
var target := Vector3.ZERO


func lookahead(speed: float) -> float:
	return clampf(min_lookahead + gain * speed, min_lookahead, max_lookahead)


## Steering angle in radians, positive = left.
func steer(rear_axle: Vector3, heading: Vector3, speed: float, path: RefPath, s_car: float) -> float:
	var ld := lookahead(absf(speed))
	target = path.point_at(s_car + ld)
	var to := target - rear_axle
	to.y = 0.0
	var h := Vector3(heading.x, 0.0, heading.z).normalized()
	if to.length() < 0.5:
		return 0.0
	var alpha := atan2(h.cross(to).y, h.dot(to))
	return atan2(2.0 * wheelbase * sin(alpha), to.length())
