from heapq import heappop, heappush
from math import inf
from .energy import evaluate_edge
from .models import PlanResult

class EWRPlanner:
    def __init__(self, graph, wind, uav, config, power_model):
        self.graph = graph
        self.wind = wind
        self.uav = uav
        self.config = config
        self.power_model = power_model

    def _wind_for(self, edge):
        key = (edge.src, edge.dst)
        if callable(self.wind):
            return self.wind(edge)
        return self.wind[key]

    def _return_energy(self, start, depot):
        if start == depot:
            return 0.0
        queue = [(0.0, start)]
        best = {start: 0.0}
        while queue:
            current, node = heappop(queue)
            if current != best.get(node):
                continue
            if node == depot:
                return current
            for edge in self.graph.outgoing(node):
                wind = self._wind_for(edge)
                evaluation = evaluate_edge(
                    graph=self.graph,
                    edge=edge,
                    wind=wind,
                    uav=self.uav,
                    planner_config=self.config,
                    power_model=self.power_model,
                    payload_kg=0.0,
                )
                if not evaluation.flyable:
                    continue
                candidate = current + evaluation.energy_wh
                if candidate < best.get(edge.dst, inf):
                    best[edge.dst] = candidate
                    heappush(queue, (candidate, edge.dst))
        return inf

    def plan(self, start, goal, depot, remaining_energy_wh, payload_kg, soc_error_wh=0.0, phase_risk=0.0):
        queue = [(0.0, 0.0, start, (start,), inf)]
        best = {(start, 0): 0.0}
        best_node_objective = {start: 0.0}
        while queue:
            objective, energy_used, node, path, min_margin = heappop(queue)
            if objective > best_node_objective.get(node, inf):
                continue
            if node == goal:
                return PlanResult(
                    path=path,
                    objective=objective,
                    energy_used_wh=energy_used,
                    minimum_margin_wh=min_margin if min_margin < inf else remaining_energy_wh,
                    aborted=False,
                    reason="feasible",
                )
            for edge in self.graph.outgoing(node):
                wind = self._wind_for(edge)
                evaluation = evaluate_edge(
                    graph=self.graph,
                    edge=edge,
                    wind=wind,
                    uav=self.uav,
                    planner_config=self.config,
                    power_model=self.power_model,
                    payload_kg=payload_kg,
                    soc_error_wh=soc_error_wh,
                    phase_risk=phase_risk,
                )
                if not evaluation.flyable:
                    continue
                return_energy = self._return_energy(edge.dst, depot)
                if return_energy == inf:
                    continue
                cumulative_energy = energy_used + evaluation.energy_wh
                margin = (
                    remaining_energy_wh
                    - cumulative_energy
                    - return_energy
                    - evaluation.safety_margin_wh
                    - evaluation.wind_uncertainty_wh
                )
                if margin < 0.0:
                    continue
                candidate_objective = objective + evaluation.total_cost
                if candidate_objective >= best_node_objective.get(edge.dst, inf):
                    continue
                best_node_objective[edge.dst] = candidate_objective
                heappush(
                    queue,
                    (
                        candidate_objective,
                        cumulative_energy,
                        edge.dst,
                        path + (edge.dst,),
                        min(min_margin, margin),
                    ),
                )
        return PlanResult(
            path=(start,),
            objective=inf,
            energy_used_wh=0.0,
            minimum_margin_wh=-inf,
            aborted=True,
            reason="safe_abort",
        )
