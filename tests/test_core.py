from ewr.config import PlannerConfig, UAVParameters
from ewr.energy import  evaluate_edge
from ewr.models import WindEstimate
from ewr.planner import EWRPlanner
from ewr.simulation import build_grid_graph, synthetic_wind_snapshot

def test_excessive_crosswind_is_rejected():
    graph = build_grid_graph(size=2, spacing_m=100.0)
    edge = graph.outgoing("n0_0")[0]
    wind = WindEstimate(vector_mps=(0.0, 20.0), epsilon_mps=1.0)
    result = evaluate_edge(
        graph,
        edge,
        wind,
        UAVParameters(),
        PlannerConfig(),
        DemoPowerModel(),
        payload_kg=0.5,
    )
    assert result.flyable is False

def test_demo_planner_returns_feasible_path():
    graph = build_grid_graph(size=4, spacing_m=120.0)
    wind = synthetic_wind_snapshot(graph, time_s=55.0, seed=3)
    planner = EWRPlanner(
        graph=graph,
        wind=wind,
        uav=UAVParameters(),
        config=PlannerConfig(),
        power_model=DemoPowerModel(),
    )
    result = planner.plan(
        start="n0_0",
        goal="n3_3",
        depot="n0_0",
        remaining_energy_wh=100.0,
        payload_kg=0.5,
    )
    assert result.aborted is False
    assert result.path[0] == "n0_0"
    assert result.path[-1] == "n3_3"
    assert result.minimum_margin_wh >= 0.0
