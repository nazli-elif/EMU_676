# UAV Pipeline Inspection — MILP & Genetic Algorithm

A combinatorial optimization framework for UAV-based pipeline inspection with mobile recharging docks. The problem jointly optimizes UAV routing, docking station selection, and battery recharging schedules to minimize total operational cost.

---

## Problem Description

Given a set of pipeline nodes, candidate docking stations, and a fleet of UAVs departing from a central depot, the goal is to:

- Cover every pipeline node exactly once
- Select which physical docking stations to open (fixed cost)
- Route each UAV such that battery constraints are never violated
- Minimize total travel cost + dock opening cost

Each UAV starts with a full battery and may recharge at opened docking stations. The model uses **node-splitting** to allow multiple visits to the same physical dock. A minimum-workload constraint ensures every active UAV visits at least one pipeline node.

---

## Methods

### Mathematical Model (MILP)
- Solved with **IBM ILOG CPLEX 12.10** via OPL
- MTZ-based subtour elimination over an expanded node set `V'`
- Full-recharge policy at docking stations
- Run via `.bat` scripts for batch execution

### Genetic Algorithm (GA)
- Population size: 80 | Generations: 400 | Mutation rate: 0.40
- Elite size: 8 | Tournament size: 5
- Local search every 25 generations
- Stagnation restart after 80 non-improving generations
- 7 independent seeds per instance: `{42, 7, 13, 99, 2024, 31, 55}`

---

## Repository Structure

```
EMU_676/
├── cplex_code/
│   ├── adim_2_v2.mod          # CPLEX OPL model
│   ├── adim_2_v2.dat          # Default data file
│   ├── adim_2_v2.ops          # CPLEX solver options
│   ├── S1.dat                 # Small instance 1  (P=3, D=2, K=2, Q=40)
│   ├── S2.dat                 # Small instance 2  (P=4, D=2, K=2, Q=40)
│   ├── S3.dat                 # Small instance 3  (P=4, D=3, K=2, Q=40)
│   ├── M1.dat                 # Medium instance 1 (P=6, D=3, K=2, Q=60)
│   ├── M2.dat                 # Medium instance 2 (P=8, D=4, K=3, Q=60)
│   ├── M3.dat                 # Medium instance 3 (P=10,D=4, K=3, Q=70)
│   ├── L1.dat                 # Large instance 1  (P=12,D=5, K=4, Q=80)
│   ├── L2.dat                 # Large instance 2  (P=15,D=5, K=4, Q=80)
│   ├── L3.dat                 # Large instance 3  (P=20,D=6, K=5, Q=100)
│   ├── run_all_cplex.bat      # Batch runner — S/M instances
│   ├── run_all_cplex_L2.bat   # Batch runner — L2 (30-min limit)
│   └── run_all_cplex_L3.bat   # Batch runner — L3 (30-min limit)
│
└── python_code/
    ├── dogrufixedbataryafull.py   # Core GA implementation
    ├── all_instances.py           # Instance definitions
    ├── run_all_instances.py       # GA batch runner (all 9 instances, 7 seeds)
    └── GA_Sonuc.txt               # GA results log
```

---

## Benchmark Instances

| Group  | Instance | Pipelines | Docks | UAVs | Battery |
|--------|----------|-----------|-------|------|---------|
| Small  | S1       | 3         | 2     | 2    | 40      |
| Small  | S2       | 4         | 2     | 2    | 40      |
| Small  | S3       | 4         | 3     | 2    | 40      |
| Medium | M1       | 6         | 3     | 2    | 60      |
| Medium | M2       | 8         | 4     | 3    | 60      |
| Medium | M3       | 10        | 4     | 3    | 70      |
| Large  | L1       | 12        | 5     | 4    | 80      |
| Large  | L2       | 15        | 5     | 4    | 80      |
| Large  | L3       | 20        | 6     | 5    | 100     |

---

## Results Summary

| Group  | Avg CPLEX Obj | Avg GA Obj | Avg Gap (%) | CPLEX Optimal |
|--------|---------------|------------|-------------|---------------|
| Small  | 116.04        | 135.16     | +16.16%     | 3 / 3         |
| Medium | 234.06        | 296.86     | +24.99%     | 3 / 3 ✓       |
| Large  | 480.17        | 626.47     | +29.50%     | 0 / 3 †       |

† 30-minute time limit applied; residual MIP gaps: L1 22.86%, L2 16.78%, L3 25.72%.

CPLEX solves all small and medium instances to **proven optimality** in under 3 minutes. For large instances, the GA is the only practical method, delivering feasible solutions in under 8 minutes (7 seeds total) with a coefficient of variation below 4%.

---

## Requirements

**MILP solver**
- IBM ILOG CPLEX 12.10+ with OPL Studio

**Genetic Algorithm**
- Python 3.8+
- No external dependencies beyond the standard library

---

## Usage

**Run CPLEX (single instance):**
```bash
oplrun cplex_code/adim_2_v2.mod cplex_code/S1.dat
```

**Run CPLEX (batch — small & medium):**
```bash
cd cplex_code
run_all_cplex.bat
```

**Run GA (all instances, 7 seeds each):**
```bash
cd python_code
python run_all_instances.py
```

Results are printed to console and saved in `python_code/GA_Sonuc.txt`.

---

## License

MIT License — see `LICENSE` for details.
