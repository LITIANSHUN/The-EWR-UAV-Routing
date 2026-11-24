from dataclasses import dataclass
from math import hypot
from .models import EdgeEvaluation
from .wind import decompose_wind, estimated_ground_speed, reachable

@dataclass(frozen=True)
class PowerModel:
    headwind_gain_w_per_mps: float = 8.0
    tailwind_credit_w_per_mps: float = 3.0
    crosswind_gain_w_per_mps2: float = 1.2
    payload_gain_w_per_kg: float = 30.0

    def components(self, parallel_wind_mps, crosswind_mps, payload_kg):
        headwind = self.headwind_gain_w_per_mps * max(0.0, -parallel_wind_mps)
        tailwind = self.tailwind_credit_w_per_mps * max(0.0, parallel_wind_mps)
        parallel = headwind - tailwind
        perpendicular = self.crosswind_gain_w_per_mps2 * crosswind_mps * crosswind_mps
        payload = self.payload_gain_w_per_kg * max(0.0, payload_kg)
        return parallel, perpendicular, payload

def evaluate_edge(graph, edge, wind, uav, planner_config, power_model, payload_kg, soc_error_wh=0.0, phase_risk=0.0):
    tangent = graph.tangent(edge)
    parallel, crosswind = decompose_wind(wind.vector_mps, tangent)
    ground_speed = estimated_ground_speed(uav.airspeed_mps, parallel)
    flyable = reachable(
        airspeed_mps=uav.airspeed_mps,
        crosswind_mps=crosswind,
        epsilon_mps=wind.epsilon_mps,
        crosswind_margin=uav.crosswind_margin,
        ground_speed_mps=ground_speed,
    )
    if not flyable:
        return EdgeEvaluation(
            flyable=False,
            ground_speed_mps=ground_speed,
            parallel_wind_mps=parallel,
            crosswind_mps=crosswind,
            energy_wh=float("inf"),
            unc_penalty=float("inf"),
            total_cost=float("inf"),
            safety_margin_wh=float("inf"),
            wind_unc_wh=float("inf"),
        )
    p_parallel, p_perp, p_load = power_model.components(parallel, crosswind, payload_kg)
    power_w = max(1.0, uav.cruise_power_w + p_parallel + p_perp + p_load)
    travel_time_s = edge.length_m / ground_speed
    energy_wh = power_w * travel_time_s / 3600.0
    prev = wind.previous_vector_mps
    drift = hypot(wind.vector_mps[0] - prev[0], wind.vector_mps[1] - prev[1])
    unc = (
        planner_config.beta_w * wind.epsilon_mps
        + planner_config.beta_tau * wind.delay_s
        + planner_config.beta_g * drift
    )
    total_cost = energy_wh + planner_config.risk_lambda * unc
    safety_margin = (
        uav.emergency_reserve_wh
        + planner_config.alpha_w * wind.sigma_energy_wh
        + planner_config.alpha_b * soc_error_wh
        + planner_config.alpha_p * phase_risk
    )
    wind_unc_wh = wind.lipschitz_wh_per_mps * wind.epsilon_mps
    return EdgeEvaluation(
        flyable=True,
        ground_speed_mps=ground_speed,
        parallel_wind_mps=parallel,
        crosswind_mps=crosswind,
        energy_wh=energy_wh,
        unc_penalty=unc,
        total_cost=total_cost,
        safety_margin_wh=safety_margin,
        wind_unc_wh=wind_unc_wh,
    )
