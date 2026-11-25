class_name RoutePlanner
extends RefCounted

## A* over the TrafficNetwork lane graph.
##
## Nodes are lanes. A car can continue onto a successor lane (cost: the length
## driven) or change to a parallel neighbour lane (a fixed penalty), the same
## edges NPC traffic uses. Left turns NPCs may not take at shared-phase signals
## are skipped too, so the ego never needs to judge gaps in oncoming traffic.
## Heuristic: straight-line distance to the goal, which never overestimates.

const LANE_CHANGE_COST := 30.0

var network: TrafficNetwork
var _preds := {} # RoadLane -> Array[RoadLane]


func _init(net: TrafficNetwork) -> void:
	network = net
	for lane in network.lanes:
		for s in network.successors(lane):
			_preds.get_or_add(s.lane, []).append(lane)


## Lanes that feed into `lane`.
func predecessors(lane: RoadLane) -> Array:
	return _preds.get(lane, [])


## True if lane changes are allowed on `lane` (not inside junctions).
func can_change_on(lane: RoadLane) -> bool:
	return not lane.has_meta("turn") and network._junction_of(lane) == null \
			and network.lane_length(lane) > 25.0


## Route from (start_lane, start_offset) to (goal_lane, goal_offset).
## Returns Array of steps {lane, entry, kind} with kind "start", "next" or
## "change"; empty if the goal can't be reached.
func plan(start_lane: RoadLane, start_offset: float, goal_lane: RoadLane, goal_offset: float) -> Array:
	var goal_pos: Vector3 = network.sample(goal_lane, goal_offset)[0]
	var g := {start_lane: 0.0}
	var entry := {start_lane: start_offset}
	var came := {}
	var open := [start_lane]
	var closed := {}
	while not open.is_empty():
		# The graph is a few hundred lanes: a linear scan beats a heap here.
		var best_i := 0
		var best_f := INF
		for i in open.size():
			var l: RoadLane = open[i]
			var f: float = g[l] + network.sample(l, entry[l])[0].distance_to(goal_pos)
			if f < best_f:
				best_f = f
				best_i = i
		var lane: RoadLane = open[best_i]
		open.remove_at(best_i)
		if lane == goal_lane and entry[lane] <= goal_offset + 0.5:
			return _unwind(came, lane, entry)
		closed[lane] = true
		var rest: float = network.lane_length(lane) - entry[lane]
		var edges := []
		for s in network.npc_successors(lane):
			edges.append({lane = s.lane, entry = s.offset, cost = rest, kind = "next"})
		if can_change_on(lane):
			for n in network.neighbours(lane):
				var off: float = n.lane.curve.get_closest_offset(
						n.lane.to_local(network.sample(lane, entry[lane])[0]))
				edges.append({lane = n.lane, entry = off, cost = LANE_CHANGE_COST, kind = "change"})
		for e in edges:
			if e.lane in closed:
				continue
			var cost: float = g[lane] + e.cost
			if cost < g.get(e.lane, INF):
				g[e.lane] = cost
				entry[e.lane] = e.entry
				came[e.lane] = {from = lane, kind = e.kind}
				if not e.lane in open:
					open.append(e.lane)
	return []


func _unwind(came: Dictionary, lane: RoadLane, entry: Dictionary) -> Array:
	var steps := []
	var l := lane
	while l in came:
		steps.push_front({lane = l, entry = entry[l], kind = came[l].kind})
		l = came[l].from
	steps.push_front({lane = l, entry = entry[l], kind = "start"})
	return steps


## Total length of a route in meters (lane changes count as zero).
func route_length(steps: Array, goal_offset: float) -> float:
	var total := 0.0
	for i in steps.size():
		var from: float = steps[i].entry
		var to: float = goal_offset if i == steps.size() - 1 else network.lane_length(steps[i].lane)
		if i + 1 < steps.size() and steps[i + 1].kind == "change":
			continue
		total += maxf(to - from, 0.0)
	return total
