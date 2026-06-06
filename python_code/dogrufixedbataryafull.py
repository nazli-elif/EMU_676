"""
UAV Pipeline Inspection - Genetic Algorithm v3
===============================================
Degisiklikler (v2 -> v3):
  Batarya sarj modeli guncellendi: dock ziyaretinde UAV bataryasi
  R kadar artmak yerine Q'ya (tam kapasiteye) yukselir.
  Bu sayede ayni dock'a ardi ardina gitmenin hicbir avantaji kalmaz;
  GA artik bu sacma rotalari secmez.

  Degistirilen yerler:
    - simulate_route        : battery = Q  (R ekleme kaldirildi)
    - build_route (before)  : bat_at_dock = Q
    - build_route (after)   : bat_at_dock = Q
    - print_solution        : gosterim guncellendi  D[arr->Q]
    - ox_crossover          : get_pipes duplicate pipeline bug duzeltildi

  MILP karsiligi (kisit 7b):
    Eski : e_jk <= e_ik - d_ij*x_ijk + R*z_jk + M(1-x_ijk)
    Yeni : e_jk <= Q*z_jk + M(1-x_ijk)
    (dock'a varinca e_jk = Q'ya sabitlenir)

  CPLEX optimal (ayni data seti): 116.03
"""

import random, math, time, itertools
from dataclasses import dataclass
from typing import List, Optional

# ──────────────────────────────────────────────────────────────────────────────
# Instance
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class Instance:
    n_nodes:     int
    n_uav:       int
    n_pipeline:  int
    n_docks:     int
    dist:        List[List[float]]
    travel_cost: float
    Q:           float
    R:           float
    q:           List[float]
    fixed_cost:  List[float]

    @property
    def depot(self):     return 0
    @property
    def pipelines(self): return list(range(1, self.n_pipeline + 1))
    @property
    def docks(self):
        return list(range(self.n_pipeline + 1,
                          self.n_pipeline + self.n_docks + 1))
    def is_dock(self, i):
        return self.n_pipeline < i <= self.n_pipeline + self.n_docks
    def is_pipeline(self, i):
        return 1 <= i <= self.n_pipeline


def build_sample_instance() -> Instance:
    dist = [
        [  0.00, 11.18, 20.10, 31.62, 25.00, 14.42, 25.50, 24.08],
        [ 11.18,  0.00, 10.20, 20.62, 15.13,  3.61, 15.13, 13.93],
        [ 20.10, 10.20,  0.00, 10.77, 18.44,  8.06,  5.10, 14.87],
        [ 31.62, 20.62, 10.77,  0.00, 18.03, 18.44,  5.39, 14.56],
        [ 25.00, 15.13, 18.44, 18.03,  0.00, 12.37, 17.69,  4.47],
        [ 14.42,  3.61,  8.06, 18.44, 12.37,  0.00, 13.15, 10.63],
        [ 25.50, 15.13,  5.10,  5.39, 17.69, 13.15,  0.00, 13.45],
        [ 24.08, 13.93, 14.87, 14.56,  4.47, 10.63, 13.45,  0.00],
    ]
    return Instance(
        n_nodes=8, n_uav=2, n_pipeline=4, n_docks=3,
        dist=dist, travel_cost=1.0, Q=40.0, R=30.0,
        q=[0.0, 3.0, 3.0, 3.0, 3.0, 0.0, 0.0, 0.0],
        fixed_cost=[0.0, 0.0, 0.0, 0.0, 0.0, 20.0, 15.0, 30.0]
    )


# ──────────────────────────────────────────────────────────────────────────────
# Solution
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class Solution:
    routes:  List[List[int]]
    fitness: float = float("inf")

    def copy(self):
        return Solution(routes=[r[:] for r in self.routes],
                        fitness=self.fitness)


# ──────────────────────────────────────────────────────────────────────────────
# Battery simulation
# ──────────────────────────────────────────────────────────────────────────────

def simulate_route(route, inst):
    battery, prev, cost, deficit = inst.Q, inst.depot, 0.0, 0.0
    for node in route:
        d     = inst.dist[prev][node]
        cost += inst.travel_cost * d
        if inst.is_dock(node):
            arrival = battery - d
            if arrival < 0: deficit += abs(arrival); arrival = 0.0
            battery = inst.Q  # dock her zaman tam doldurur
        else:
            battery -= d + inst.q[node]
            if battery < 0: deficit += abs(battery); battery = 0.0
        prev = node
    ret   = inst.dist[prev][inst.depot]
    cost += inst.travel_cost * ret
    battery -= ret
    if battery < 0: deficit += abs(battery)
    return cost, battery, deficit


def evaluate(sol, inst):
    total_cost, total_deficit, opened = 0.0, 0.0, set()
    for route in sol.routes:
        if not route: continue
        cost, _, deficit = simulate_route(route, inst)
        total_cost    += cost
        total_deficit += deficit
        for n in route:
            if inst.is_dock(n): opened.add(n)
    for d in opened: total_cost += inst.fixed_cost[d]
    return total_cost, total_deficit < 1e-6


def penalized_fitness(sol, inst, penalty=500.0, idle_penalty=300.0):
    total_cost, total_deficit, opened = 0.0, 0.0, set()
    for route in sol.routes:
        if not route: continue
        cost, _, deficit = simulate_route(route, inst)
        total_cost    += cost
        total_deficit += deficit
        for n in route:
            if inst.is_dock(n): opened.add(n)
    for d in opened: total_cost += inst.fixed_cost[d]
    idle = sum(1 for r in sol.routes
               if not any(inst.is_pipeline(n) for n in r))
    return total_cost + penalty * total_deficit + idle_penalty * idle


# ──────────────────────────────────────────────────────────────────────────────
# Route builder  (supports dock-before AND dock-after)
# ──────────────────────────────────────────────────────────────────────────────

def build_route(pipes: List[int], inst: Instance,
                allowed_docks: Optional[List[int]] = None) -> List[int]:
    """
    Build a feasible route from an ordered pipeline list.
    At each step, if battery is insufficient:
      - try inserting a dock BEFORE the pipeline node (prev->dock->node)
      - if that fails, try inserting a dock AFTER  (node->dock->next)
    allowed_docks: restrict which docks can be used (None = all docks)
    """
    docks = allowed_docks if allowed_docks is not None else inst.docks
    result  = []
    battery = inst.Q
    prev    = inst.depot

    for idx, node in enumerate(pipes):
        d_pn      = inst.dist[prev][node]
        bat_after = battery - d_pn - inst.q[node]
        min_needed_after = inst.dist[node][inst.depot]  # conservative

        # ── try dock BEFORE node ──────────────────────────────────────────
        if bat_after < 0 or bat_after < min_needed_after:
            best_pre = None
            best_pre_score = float("inf")
            for d in docks:
                d_pd        = inst.dist[prev][d]
                d_dn        = inst.dist[d][node]
                bat_at_dock = battery - d_pd  # varis bataryasi
                if bat_at_dock < 0: continue
                bat_at_dock = inst.Q           # tam sarj
                bat_at_node = bat_at_dock - d_dn - inst.q[node]
                if bat_at_node < 0: continue
                if bat_at_node < inst.dist[node][inst.depot]: continue
                score = (d_pd + d_dn - d_pn) + inst.fixed_cost[d]
                if score < best_pre_score:
                    best_pre_score = score
                    best_pre = d

            if best_pre is not None:
                result.append(best_pre)
                arr = battery - inst.dist[prev][best_pre]
                if arr < 0: arr = 0.0
                battery = inst.Q  # tam sarj
                prev = best_pre

        # visit node
        result.append(node)
        battery -= inst.dist[prev][node] + inst.q[node]
        battery  = max(battery, 0.0)
        prev = node

        # ── try dock AFTER node if still can't return ─────────────────────
        can_return = battery >= inst.dist[prev][inst.depot]
        if not can_return:
            best_post = None
            best_post_score = float("inf")
            for d in docks:
                d_nd        = inst.dist[node][d]
                bat_at_dock = battery - d_nd
                if bat_at_dock < 0: continue
                bat_at_dock = inst.Q  # tam sarj
                if bat_at_dock < inst.dist[d][inst.depot]: continue
                score = d_nd + inst.fixed_cost[d]
                if score < best_post_score:
                    best_post_score = score
                    best_post = d

            if best_post is not None:
                result.append(best_post)
                battery = inst.Q  # tam sarj
                prev = best_post

    return result


def greedy_insert_dock(pipes, inst):
    return build_route(pipes, inst, allowed_docks=None)


# ──────────────────────────────────────────────────────────────────────────────
# Initial solution
# ──────────────────────────────────────────────────────────────────────────────

def build_greedy_solution(inst, forced_docks=None):
    pipes = inst.pipelines[:]
    random.shuffle(pipes)
    chunks = [[] for _ in range(inst.n_uav)]
    for k in range(min(inst.n_uav, len(pipes))):
        chunks[k].append(pipes[k])
    for i in range(inst.n_uav, len(pipes)):
        chunks[i % inst.n_uav].append(pipes[i])

    docks_to_use = forced_docks if forced_docks else None
    routes = []
    for chunk in chunks:
        if not chunk:
            routes.append([])
            continue
        ordered, remaining, cur = [], chunk[:], inst.depot
        while remaining:
            nxt = min(remaining, key=lambda n: inst.dist[cur][n])
            ordered.append(nxt); remaining.remove(nxt); cur = nxt
        routes.append(build_route(ordered, inst, docks_to_use))

    sol = Solution(routes=routes)
    sol.fitness = penalized_fitness(sol, inst)
    return sol


# ──────────────────────────────────────────────────────────────────────────────
# Exhaustive dock combination local search
# ──────────────────────────────────────────────────────────────────────────────

def exhaustive_dock_ls(sol: Solution, inst: Instance) -> Solution:
    """
    For each route's pipeline ordering, try every non-empty subset of docks.
    Keep the globally cheapest feasible combination.
    """
    best = sol.copy()

    for ri in range(inst.n_uav):
        pipes = [n for n in best.routes[ri] if inst.is_pipeline(n)]
        if not pipes:
            continue

        for r in range(0, len(inst.docks) + 1):
            subsets = (itertools.combinations(inst.docks, r)
                       if r > 0 else [()])
            for subset in subsets:
                route = build_route(pipes, inst, list(subset))
                if not route:
                    continue
                new_routes     = [rr[:] for rr in best.routes]
                new_routes[ri] = route
                cand           = Solution(routes=new_routes)
                cand.fitness   = penalized_fitness(cand, inst)
                if cand.fitness < best.fitness - 1e-6:
                    best = cand

    return best


def build_diverse_population(inst, size):
    population = []

    for r in range(1, len(inst.docks) + 1):
        for subset in itertools.combinations(inst.docks, r):
            for _ in range(2):
                sol = build_greedy_solution(inst, forced_docks=list(subset))
                population.append(sol)

    while len(population) < size:
        population.append(build_greedy_solution(inst))

    population.sort(key=lambda s: s.fitness)
    improved = []
    for s in population[:min(len(population), 15)]:
        improved.append(exhaustive_dock_ls(s, inst))
    population[:len(improved)] = improved
    population.sort(key=lambda s: s.fitness)

    return population[:size]


# ──────────────────────────────────────────────────────────────────────────────
# Crossover
# ──────────────────────────────────────────────────────────────────────────────

def ox_crossover(p1, p2, inst):
    def get_pipes(sol):
        seen, result = set(), []
        for r in sol.routes:
            for n in r:
                if inst.is_pipeline(n) and n not in seen:
                    seen.add(n); result.append(n)
        return result

    def build(pipes):
        n     = inst.n_uav
        chunk = max(1, math.ceil(len(pipes) / n))
        routes = []
        for k in range(n):
            seg = pipes[k * chunk:(k + 1) * chunk]
            routes.append(greedy_insert_dock(seg, inst) if seg else [])
        s = Solution(routes=routes)
        s.fitness = penalized_fitness(s, inst)
        return s

    pp1, pp2 = get_pipes(p1), get_pipes(p2)
    if len(pp1) != len(pp2) or set(pp1) != set(pp2):
        return p1.copy(), p2.copy()
    n = len(pp1)
    if n < 2: return p1.copy(), p2.copy()
    a, b = sorted(random.sample(range(n), 2))

    def ox(pa, pb):
        child = [None] * n
        child[a:b + 1] = pa[a:b + 1]
        seg  = set(pa[a:b + 1])
        fill = [x for x in pb if x not in seg]
        fi   = 0
        for i in range(n):
            if child[i] is None:
                child[i] = fill[fi]; fi += 1
        return child

    return build(ox(pp1, pp2)), build(ox(pp2, pp1))


# ──────────────────────────────────────────────────────────────────────────────
# Mutation
# ──────────────────────────────────────────────────────────────────────────────

def mutate(sol, inst, rate=0.40):
    if random.random() > rate: return sol.copy()
    new = sol.copy()
    op  = random.randint(0, 4)

    all_pipes = [(ri, i, n)
                 for ri, r in enumerate(new.routes)
                 for i, n  in enumerate(r) if inst.is_pipeline(n)]
    if not all_pipes: return new

    if op == 0 and len(all_pipes) >= 2:
        (r1, i1, _), (r2, i2, _) = random.sample(all_pipes, 2)
        new.routes[r1][i1], new.routes[r2][i2] = \
            new.routes[r2][i2], new.routes[r1][i1]
    elif op == 1:
        r1, i1, node = random.choice(all_pipes)
        new.routes[r1].pop(i1)
        r2  = random.randint(0, inst.n_uav - 1)
        pos = random.randint(0, len(new.routes[r2]))
        new.routes[r2].insert(pos, node)
    elif op == 2:
        ri = random.choice(list({r for r, _, _ in all_pipes}))
        route = new.routes[ri]
        if len(route) >= 4:
            i = random.randint(0, len(route) - 3)
            j = random.randint(i + 2, len(route) - 1)
            route[i:j + 1] = route[i:j + 1][::-1]
    elif op == 3:
        by_route = {}
        for ri, idx, n in all_pipes: by_route.setdefault(ri, []).append((idx, n))
        ri = random.choice(list(by_route.keys()))
        pair = by_route[ri]
        if len(pair) >= 2:
            k      = random.randint(0, len(pair) - 2)
            p0, n0 = pair[k]; p1, n1 = pair[k + 1]
            for pos in sorted([p0, p1], reverse=True): new.routes[ri].pop(pos)
            r2  = random.randint(0, inst.n_uav - 1)
            ins = random.randint(0, len(new.routes[r2]))
            new.routes[r2].insert(ins, n1); new.routes[r2].insert(ins, n0)
    elif op == 4:
        ri = random.randint(0, inst.n_uav - 1)
        dock_idx = [i for i, n in enumerate(new.routes[ri]) if inst.is_dock(n)]
        if dock_idx: new.routes[ri].pop(random.choice(dock_idx))

    for ri in range(inst.n_uav):
        pipes_only     = [n for n in new.routes[ri] if inst.is_pipeline(n)]
        new.routes[ri] = greedy_insert_dock(pipes_only, inst)

    new.fitness = penalized_fitness(new, inst)
    return new


# ──────────────────────────────────────────────────────────────────────────────
# Local search
# ──────────────────────────────────────────────────────────────────────────────

def two_opt_ls(sol, inst, max_no_imp=30):
    best = sol.copy()
    for ri in range(inst.n_uav):
        pipes = [n for n in best.routes[ri] if inst.is_pipeline(n)]
        if len(pipes) < 3: continue
        no_imp = 0
        while no_imp < max_no_imp:
            improved = False
            for i in range(len(pipes) - 1):
                for j in range(i + 2, len(pipes)):
                    np2  = pipes[:i+1] + list(reversed(pipes[i+1:j+1])) + pipes[j+1:]
                    nr   = greedy_insert_dock(np2, inst)
                    cand = Solution(routes=[r if k != ri else nr
                                            for k, r in enumerate(best.routes)])
                    cand.fitness = penalized_fitness(cand, inst)
                    if cand.fitness < best.fitness - 1e-6:
                        best = cand; pipes = np2; improved = True
            no_imp = 0 if improved else no_imp + 1
    return best


def relocate_ls(sol, inst):
    best = sol.copy()
    all_pipes = [(ri, n) for ri, r in enumerate(best.routes)
                 for n in r if inst.is_pipeline(n)]
    random.shuffle(all_pipes)
    for ri, node in all_pipes:
        src = [n for n in best.routes[ri] if inst.is_pipeline(n)]
        if node not in src: continue
        src2 = src[:]; src2.remove(node)
        for rj in range(inst.n_uav):
            dst = [n for n in best.routes[rj] if inst.is_pipeline(n)]
            for pos in range(len(dst) + 1):
                dst2 = dst[:pos] + [node] + dst[pos:]
                new_routes     = [r[:] for r in best.routes]
                new_routes[ri] = greedy_insert_dock(src2, inst)
                new_routes[rj] = greedy_insert_dock(dst2, inst)
                cand = Solution(routes=new_routes)
                cand.fitness = penalized_fitness(cand, inst)
                if cand.fitness < best.fitness - 1e-6:
                    best = cand
    return best


# ──────────────────────────────────────────────────────────────────────────────
# Selection
# ──────────────────────────────────────────────────────────────────────────────

def tournament(population, k=5):
    return min(random.sample(population, min(k, len(population))),
               key=lambda s: s.fitness)


# ──────────────────────────────────────────────────────────────────────────────
# Genetic Algorithm
# ──────────────────────────────────────────────────────────────────────────────

def genetic_algorithm(inst, pop_size=80, n_gen=400,
                       mutation_rate=0.40, elite=8, tourn_k=5,
                       ls_every=25, penalty=500.0, idle_penalty=300.0,
                       stagnation=80, seed=42, verbose=True):
    random.seed(seed)
    t0 = time.time()

    population = build_diverse_population(inst, pop_size)
    best_ever  = population[0].copy()
    history    = []
    no_imp_cnt = 0

    if verbose:
        print(f"\n{'Gen':>5} {'Fitness':>12} {'Obj':>10} {'Feas':>6} {'Time':>7}")
        print("-" * 48)

    for gen in range(n_gen):
        new_pop = [s.copy() for s in population[:elite]]
        while len(new_pop) < pop_size:
            c1, c2 = ox_crossover(tournament(population, tourn_k),
                                   tournament(population, tourn_k), inst)
            new_pop.append(mutate(c1, inst, mutation_rate))
            if len(new_pop) < pop_size:
                new_pop.append(mutate(c2, inst, mutation_rate))

        population = sorted(new_pop[:pop_size], key=lambda s: s.fitness)

        if (gen + 1) % ls_every == 0:
            top = []
            for s in population[:elite]:
                s = two_opt_ls(s, inst, max_no_imp=20)
                s = relocate_ls(s, inst)
                s = exhaustive_dock_ls(s, inst)
                top.append(s)
            population[:elite] = sorted(top, key=lambda s: s.fitness)
            population.sort(key=lambda s: s.fitness)

        cur = population[0]
        obj, feas = evaluate(cur, inst)

        if cur.fitness < best_ever.fitness - 1e-6:
            best_ever = cur.copy(); no_imp_cnt = 0
        else:
            no_imp_cnt += 1

        if no_imp_cnt >= stagnation:
            keep  = population[:elite]
            fresh = build_diverse_population(inst, pop_size - elite)
            population = sorted(keep + fresh, key=lambda s: s.fitness)
            no_imp_cnt = 0
            if verbose: print(f"  [diversification restart at gen {gen}]")

        history.append((gen, cur.fitness, obj, feas))
        elapsed = time.time() - t0
        if verbose and (gen % 50 == 0 or gen == n_gen - 1):
            print(f"{gen:>5} {cur.fitness:>12.4f} {obj:>10.4f} "
                  f"{'YES' if feas else 'NO':>6} {elapsed:>6.1f}s")

    return best_ever, history


# ──────────────────────────────────────────────────────────────────────────────
# Printer
# ──────────────────────────────────────────────────────────────────────────────

def print_solution(sol, inst, cplex_opt=116.03):
    obj, feas = evaluate(sol, inst)
    opened    = set()
    print("\n" + "=" * 62)
    print("  SOLUTION")
    print("=" * 62)
    print(f"  Status    : {'FEASIBLE' if feas else 'INFEASIBLE'}")
    print(f"  Objective : {obj:.4f}")
    if feas:
        print(f"  CPLEX opt : {cplex_opt:.2f}   Gap: {(obj-cplex_opt)/cplex_opt*100:.2f}%")
    print()
    for k, route in enumerate(sol.routes):
        battery = inst.Q; prev = inst.depot; steps = []
        for node in route:
            d = inst.dist[prev][node]
            if inst.is_dock(node):
                arr = battery - d
                battery = inst.Q  # tam sarj
                steps.append(f"D{node}[arr={arr:.1f}->Q]")
                opened.add(node)
            else:
                battery -= d + inst.q[node]
                steps.append(f"P{node}[{battery:.1f}]")
            prev = node
        ret = inst.dist[prev][inst.depot]; battery -= ret
        tc  = sum(inst.dist[([inst.depot]+route)[i]][n]
                  for i, n in enumerate(route+[inst.depot]))
        status = "OK" if battery >= -1e-6 else "DEFICIT"
        print(f"  UAV {k} [{status}]")
        print(f"    {' -> '.join(['0']+steps+[f'0(ret={battery:.2f})'])}")
        print(f"    travel = {tc:.2f}")
        print()
    travel = sum(inst.travel_cost * inst.dist[([inst.depot]+r)[i]][n]
                 for r in sol.routes for i, n in enumerate(r+[inst.depot]))
    dock_c = sum(inst.fixed_cost[d] for d in opened)
    print(f"  Opened docks : {sorted(opened)}  (fixed cost {dock_c:.1f})")
    print(f"  Travel cost  : {travel:.4f}")
    print(f"  Dock cost    : {dock_c:.4f}")
    print(f"  Total        : {obj:.4f}")
    print("=" * 62)


# ──────────────────────────────────────────────────────────────────────────────
# Multi-seed
# ──────────────────────────────────────────────────────────────────────────────

def multi_seed_run(inst, seeds=(42,7,13,99,2024,31,55),
                   cplex_opt=116.03, **ga_kw):
    best_overall = None
    print(f"\n{'Seed':>6} {'Objective':>12} {'Gap%':>8} {'Feas':>6}")
    print("-" * 38)
    for seed in seeds:
        best, _ = genetic_algorithm(inst, seed=seed, verbose=False, **ga_kw)
        obj, feas = evaluate(best, inst)
        gap = (obj - cplex_opt) / cplex_opt * 100 if feas else float("inf")
        print(f"{seed:>6} {obj:>12.4f} {gap:>7.2f}%  {'YES' if feas else 'NO'}")
        if best_overall is None or best.fitness < best_overall.fitness:
            best_overall = best
    return best_overall


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("UAV Pipeline Inspection - Genetic Algorithm v3")
    print("=" * 50)
    inst = build_sample_instance()
    print(f"Instance : {inst.n_nodes} nodes | {inst.n_pipeline} pipelines | "
          f"{inst.n_docks} docks | {inst.n_uav} UAVs")
    print(f"Q={inst.Q}  c={inst.travel_cost}")

    GA = dict(pop_size=80, n_gen=400, mutation_rate=0.40,
              elite=8, tourn_k=5, ls_every=25,
              penalty=500.0, idle_penalty=300.0, stagnation=80)

    print("\n--- Single run (seed=42) ---")
    best, history = genetic_algorithm(inst, seed=42, verbose=True, **GA)
    print_solution(best, inst)

    print("\n--- Multi-seed run ---")
    best_ms = multi_seed_run(inst, seeds=(42,7,13,99,2024,31,55), **GA)
    print("\nBest across all seeds:")
    print_solution(best_ms, inst)

    feas_gen = next((h[0] for h in history if h[3]), None)
    print(f"\nFirst feasible at generation : {feas_gen}")
    print(f"CPLEX optimal                : 116.03")