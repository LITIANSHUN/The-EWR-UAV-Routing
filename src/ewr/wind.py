from math import hypot

def decompose_wind(wind_vector_mps, tangent):
    wx, wy = wind_vector_mps
    tx, ty = tangent
    parallel = wx * tx + wy * ty
    px = wx - parallel * tx
    py = wy - parallel * ty
    crosswind = hypot(px, py)
    return parallel, crosswind

def estimated_ground_speed(airspeed_mps, parallel_wind_mps):
    return airspeed_mps + parallel_wind_mps

def reachable(airspeed_mps, crosswind_mps, epsilon_mps, crosswind_margin, ground_speed_mps):
    lateral_ok = crosswind_mps + epsilon_mps <= crosswind_margin * airspeed_mps
    forward_ok = ground_speed_mps - epsilon_mps > 0.0
    return lateral_ok and forward_ok
