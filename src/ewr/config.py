from dataclasses import dataclass

@dataclass(frozen=True)
class UAVParameters:
    airspeed_mps: float = 12.0
    cruise_power_w: float = 180.0
    battery_capacity_wh: float = 100.0
    emergency_reserve_wh: float = 10.0
    crosswind_margin: float = 0.75
    max_payload_kg: float = 0.5

@dataclass(frozen=True)
class PlannerConfig:
    risk_lambda: float = 2.0
    beta_w: float = 1.0
    beta_tau: float = 0.05
    beta_g: float = 0.5
    alpha_w: float = 1.0
    alpha_b: float = 1.0
    alpha_p: float = 1.0
