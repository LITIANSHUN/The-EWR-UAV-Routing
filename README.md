# EWR-UAV-Routing

Project for **Energy-Aware Wind-Resilient Routing (EWR)** in truck-assisted multi-UAV delivery under uncertain wind.

The repository is organized around the method described in the paper:

> **Energy-Aware Wind-Resilient Routing for Truck-Assisted Multi-UAV Delivery under Wind Uncertainty**  
> Tianshun Li, Yanggang Sheng, Hongliang Lu, Zhongzhen Wang, Haoang Li, Xinhu Zheng

EWR models the delivery environment as a time-dependent directed energy graph. Edge costs are updated from delayed wind observations, payload state, and uncertainty. Candidate routes are filtered by wind reachability and battery return feasibility before being accepted.

## Highlights

- Time-dependent directed energy graph
- Delayed and uncertain wind estimate
- Parallel and crosswind decomposition
  
  

## Installation

```bash
git clone https://github.com/your-name/ewr-uav-routing.git
cd ewr-uav-routing
pip install -e .
```

For development:

```bash
pip install -e ".[dev]"
```

## Quick Start

Run the synthetic demo:

```bash
python -m ewr
```

output:

```json
{
  "path": ["n0_0", "...", "n5_5"],
  "aborted": false,
  "energy_used_wh": 0.0,
  "minimum_margin_wh": 0.0
}
```

The exact numbers depend on the synthetic wind snapshot and configuration.

## Configuration

The default UAV values:

| Parameter            | Default |
| -------------------- | -------:|
| Nominal airspeed     | 12 m/s  |
| Nominal cruise power | 180 W   |
| Battery capacity     | 100 Wh  |
| Emergency reserve    | 10 Wh   |
| Crosswind margin     | 0.75    |
| Maximum payload      | 0.5 kg  |

## Data

The paper uses:

- Synthetic delivery graphs
- Time-indexed wind logs from a public ATS-based truck-UAV delivery dataset
- ASOS wind observations obtained through the Iowa Environmental Mesonet interface

**ATS preprocessing: Coming soon.**

**ASOS preprocessing: Coming soon.**

## 

## API Example

```python
from ewr import EWRPlanner, PlannerConfig, UAVParameters
from ewr.simulation import build_grid_graph, synthetic_wind_snapshot
from ewr.energy import DemoPowerModel

graph = build_grid_graph(size=6, spacing_m=150.0)
wind = synthetic_wind_snapshot(graph, time_s=80.0, seed=7)

planner = EWRPlanner(
    graph=graph,
    wind=wind,
    uav=UAVParameters(),
    config=PlannerConfig(),
    power_model=DemoPowerModel(),
)

result = planner.plan(
    start="n0_0",
    goal="n5_5",
    depot="n0_0",
    remaining_energy_wh=100.0,
    payload_kg=0.5,
)

print(result)
```

## 

## Citation

If you find this repo useful, please feel free to cite this:

```bibtex
@misc{li2026energyawarewindresilientroutingtruckassisted,
      title={Energy-Aware Wind-Resilient Routing for Truck-Assisted Multi-UAV Delivery under Wind Uncertainty}, 
      author={Tianshun Li and Yanggang Sheng and Hongliang Lu and Zhongzhen Wang and Haoang Li and Xinhu Zheng},
      year={2026},
      eprint={2608.11641},
      archivePrefix={arXiv},
      primaryClass={eess.SY},
      url={https://arxiv.org/abs/2608.11641}, 
}
```

## 
