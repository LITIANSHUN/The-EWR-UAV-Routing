from heapq import heappop, heappush
from math import inf
from .energy import evaluate_edge
from .models import WindEstimate

def shortest_path(graph, start, goal, edge_cost):
    queue = [(0.0, start, (start,))]
    best = {start: 0.0}
    while queue:
        cost, node, path = heappop(queue)
        if cost != best.get(node):
            continue
        if node == goal:
            return path, cost
        for edge in graph.outgoing(node):
            candidate = cost + edge_cost(edge)
            if candidate < best.get(edge.dst, inf):
                best[edge.dst] = candidate
                heappush(queue, (candidate, edge.dst, path + (edge.dst,)))
    return (start,), inf

def sp_no_wind(graph, start, goal):
    return shortest_path(graph, start, goal, lambda edge: edge.length_m)

def energy_sp(graph, start, goal, uav, config, power_model, payload_kg):
    zero = WindEstimate(vector_mps=(0.0, 0.0), epsilon_mps=0.0)
    def cost(edge):
        return evaluate_edge(graph, edge, zero, uav, config, power_model, payload_kg).energy_wh
    return shortest_path(graph, start, goal, cost)

def initial_wind_sp(graph, wind, start, goal, uav, config, power_model, payload_kg):
    def cost(edge):
        return evaluate_edge(graph, edge, wind[(edge.src, edge.dst)], uav, config, power_model, payload_kg).energy_wh
    return shortest_path(graph, start, goal, cost)

def online_replan(graph, wind, start, goal, uav, config, power_model, payload_kg):
    def cost(edge):
        evaluation = evaluate_edge(graph, edge, wind[(edge.src, edge.dst)], uav, config, power_model, payload_kg)
        return evaluation.energy_wh
    return shortest_path(graph, start, goal, cost)

def greedy_energy(graph, wind, start, goal, uav, config, power_model, payload_kg, max_steps=1000):
    node = start
    path = [start]
    seen = {start}
    total = 0.0
    for _ in range(max_steps):
        if node == goal:
            return tuple(path), total
        candidates = []
        for edge in graph.outgoing(node):
            evaluation = evaluate_edge(graph, edge, wind[(edge.src, edge.dst)], uav, config, power_model, payload_kg)
            if evaluation.flyable:
                candidates.append((evaluation.energy_wh, edge))
        if not candidates:
            return tuple(path), inf
        candidates.sort(key=lambda item: item[0])
        chosen_cost, chosen = candidates[0]
        if chosen.dst in seen:
            return tuple(path), inf
        total += chosen_cost
        node = chosen.dst
        path.append(node)
        seen.add(node)
    return tuple(path), inf

