class_name Scenarios
extends RefCounted

## The scripted test scenarios. Lane paths are relative to the city scene
## (node_3d.tscn). Every scenario runs in the full living city: NPC traffic,
## pedestrians (including random jaywalkers), signals on their normal cycle.
## A scenario passes when the car reaches its goal in time with no collision,
## no red-light violation, at least MIN_PED_DISTANCE from every pedestrian and
## within MAX_LANE_DEVIATION of its reference path.

const MIN_PED_DISTANCE := 1.0
const MAX_LANE_DEVIATION := 1.5

const ALL := [
	{
		name = "red_light",
		title = "Red light",
		description = "Approach a signalised T-junction that turns red ahead of the car; stop at the line, wait for green, continue.",
		start = ["RoadManager/Street_S/RP_001/pR0_nR0", 2.0],
		goal = ["RoadManager/Corner_SE/RP_003/pF0_nF0", 30.0],
		time_limit = 90.0,
	},
	{
		name = "jaywalker",
		title = "Jaywalker",
		description = "A pedestrian steps off the kerb mid-block, right in front of the car; brake, let them cross, carry on.",
		start = ["RoadManager/Cross_S1/RP_001/pR1_nR1", 2.0],
		goal = ["RoadManager/Cross_S2/RP_001/pR1_nR1", 30.0],
		time_limit = 120.0,
	},
	{
		name = "road_works",
		title = "Road works",
		description = "Cones and barriers close the right lane; change lanes around them, then turn left at the T-junction.",
		start = ["RoadManager/Ave_W3/RP_001/pR1_nR1", 2.0],
		goal = ["RoadManager/Street_S/RP_001/pR0_nR0", 40.0],
		time_limit = 120.0,
	},
	{
		name = "roundabout",
		title = "Roundabout",
		description = "Enter the roundabout from the west, yielding to circulating traffic, and leave by the north exit.",
		start = ["RoadManager/Cross_N2/RP_001/pR0_nR0", 4.0],
		goal = ["RoadManager/Ave_E1/RP_001/pF0_nF0", 35.0],
		time_limit = 120.0,
	},
	{
		name = "highway_merge",
		title = "Highway merge",
		description = "Take the on-ramp and merge into the highway's right lane between fast traffic, then cruise at 100 km/h.",
		start = ["RoadManager/Ramp_Connector/RP_001/pF0_nF0", 4.0],
		goal = ["RoadManager/HW_Loop/RP_001/pR2_nR2", 220.0],
		time_limit = 120.0,
	},
	{
		name = "cross_town",
		title = "Cross-town route",
		description = "A* route from the south-west of the city to the north-east: several signals, turns and live traffic.",
		start = ["RoadManager/Street_W/RP_001/pR0_nR0", 4.0],
		goal = ["RoadManager/Corner_NE/RP_003/pR0_nR0", 25.0],
		time_limit = 300.0,
	},
]


static func names() -> PackedStringArray:
	var out := PackedStringArray()
	for s in ALL:
		out.append(s.name)
	return out


static func get_def(scenario_name: String) -> Dictionary:
	for s in ALL:
		if s.name == scenario_name:
			return s
	return {}
