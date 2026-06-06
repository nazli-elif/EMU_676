"""
UAV Pipeline Inspection - Tüm Instance Runner
==============================================
Kullanım:
    python run_all_instances.py

Bu dosya dogrufixedbataryafull.py ve all_instances.py ile
aynı klasörde olmalıdır.

Çıktı: results.csv  (her instance için CPLEX/GA karşılaştırma tablosu)
"""

import time
import csv
import sys
from all_instances import INSTANCES
from dogrufixedbataryafull import genetic_algorithm, evaluate, print_solution

# ──────────────────────────────────────────────────────────────────────────────
# CPLEX optimal değerleri (runlardan sonra buraya girin)
# Henüz bilinmiyorsa None bırakın → gap hesaplanmaz
# ──────────────────────────────────────────────────────────────────────────────
CPLEX_OPTIMAL = {
    "S1": None,   # örn: 85.30
    "S2": None,
    "S3": 116.03, # bilinen baseline
    "M1": None,
    "M2": None,
    "M3": None,
    "L1": None,
    "L2": None,
    "L3": None,
}

# ──────────────────────────────────────────────────────────────────────────────
# GA parametreleri
# ──────────────────────────────────────────────────────────────────────────────
GA_PARAMS = dict(
    pop_size=80,
    n_gen=400,
    mutation_rate=0.40,
    elite=8,
    tourn_k=5,
    ls_every=25,
    penalty=500.0,
    idle_penalty=300.0,
    stagnation=80,
)

SEEDS = [42, 7, 13, 99, 2024, 31, 55]

# Sadece belirli instance'ları çalıştırmak istersen buraya yaz:
# RUN_ONLY = ["S1", "S2", "S3"]   # küçük örnekler
# RUN_ONLY = ["M1", "M2", "M3"]   # orta örnekler
RUN_ONLY = None  # None = hepsini çalıştır


# ──────────────────────────────────────────────────────────────────────────────
# Runner
# ──────────────────────────────────────────────────────────────────────────────

def run_instance(name, inst):
    cplex_opt = CPLEX_OPTIMAL.get(name)

    print(f"\n{'='*60}")
    print(f"  Instance: {name}  |  P={inst.n_pipeline}  D={inst.n_docks}"
          f"  K={inst.n_uav}  Q={inst.Q}")
    print(f"{'='*60}")
    print(f"{'Seed':>6}  {'Objective':>12}  {'Gap%':>8}  {'Feas':>6}  {'Time(s)':>8}")
    print("-"*50)

    seed_results = []
    best_overall = None
    t_total_start = time.time()

    for seed in SEEDS:
        t0 = time.time()
        best, history = genetic_algorithm(inst, seed=seed, verbose=False, **GA_PARAMS)
        elapsed = time.time() - t0

        obj, feas = evaluate(best, inst)
        if cplex_opt and feas:
            gap = (obj - cplex_opt) / cplex_opt * 100
            gap_str = f"{gap:>7.2f}%"
        else:
            gap = None
            gap_str = "    N/A"

        print(f"{seed:>6}  {obj:>12.4f}  {gap_str}  {'YES' if feas else 'NO':>6}  {elapsed:>7.1f}s")

        seed_results.append({
            "seed": seed, "obj": obj, "feas": feas,
            "gap": gap, "time": elapsed, "solution": best, "history": history
        })

        if best_overall is None or best.fitness < best_overall.fitness:
            best_overall = best

    t_total = time.time() - t_total_start

    # İstatistikler
    feas_results = [r for r in seed_results if r["feas"]]
    objs = [r["obj"] for r in feas_results] if feas_results else [r["obj"] for r in seed_results]
    best_obj = min(objs)
    mean_obj = sum(objs) / len(objs)
    std_obj  = (sum((x - mean_obj)**2 for x in objs) / len(objs))**0.5
    cv       = std_obj / mean_obj * 100 if mean_obj > 0 else 0

    best_gap = (best_obj - cplex_opt) / cplex_opt * 100 if (cplex_opt and feas_results) else None

    print(f"\n  Özet  →  Best: {best_obj:.4f}  Mean: {mean_obj:.4f}"
          f"  Std: {std_obj:.4f}  CV: {cv:.2f}%")
    if best_gap is not None:
        print(f"  Gap (best vs CPLEX): {best_gap:.2f}%")
    print(f"  Toplam süre: {t_total:.1f}s  |  Feasible seeds: {len(feas_results)}/{len(SEEDS)}")

    # En iyi çözümü detaylı göster
    print("\n  ── En İyi Çözüm ──")
    if cplex_opt:
        print_solution(best_overall, inst, cplex_opt=cplex_opt)
    else:
        # cplex_opt bilinmiyor: gap satırını atlamak için geçici patch
        obj, feas = evaluate(best_overall, inst)
        opened = set()
        print("\n" + "=" * 62)
        print("  SOLUTION")
        print("=" * 62)
        print(f"  Status    : {'FEASIBLE' if feas else 'INFEASIBLE'}")
        print(f"  Objective : {obj:.4f}  (CPLEX opt: henüz bilinmiyor)")
        print("=" * 62)

    return {
        "name": name,
        "n_pipeline": inst.n_pipeline,
        "n_docks": inst.n_docks,
        "n_uav": inst.n_uav,
        "Q": inst.Q,
        "cplex_opt": cplex_opt if cplex_opt else "",
        "ga_best": best_obj,
        "ga_mean": mean_obj,
        "ga_std": std_obj,
        "ga_cv": cv,
        "ga_gap_best": best_gap if best_gap is not None else "",
        "feas_seeds": len(feas_results),
        "total_seeds": len(SEEDS),
        "total_time_s": round(t_total, 1),
        "seed_details": {r["seed"]: {"obj": r["obj"], "feas": r["feas"],
                                      "gap": r["gap"], "time": round(r["time"], 1)}
                         for r in seed_results},
        "best_solution": best_overall,
        "all_histories": {r["seed"]: r["history"] for r in seed_results},
    }


def save_csv(all_results, path="results.csv"):
    fieldnames = [
        "instance", "group", "|P|", "|D|", "K", "Q",
        "cplex_opt", "ga_best", "ga_mean", "ga_std", "cv%",
        "gap_best%", "feas_seeds", "total_time_s",
        "seed_42", "seed_7", "seed_13", "seed_99", "seed_2024", "seed_31", "seed_55"
    ]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in all_results:
            group = {"S": "Small", "M": "Medium", "L": "Large"}.get(r["name"][0], "?")
            row = {
                "instance": r["name"], "group": group,
                "|P|": r["n_pipeline"], "|D|": r["n_docks"],
                "K": r["n_uav"], "Q": r["Q"],
                "cplex_opt": r["cplex_opt"],
                "ga_best": round(r["ga_best"], 4),
                "ga_mean": round(r["ga_mean"], 4),
                "ga_std":  round(r["ga_std"], 4),
                "cv%":     round(r["ga_cv"], 2),
                "gap_best%": round(r["ga_gap_best"], 2) if r["ga_gap_best"] != "" else "",
                "feas_seeds": f"{r['feas_seeds']}/{r['total_seeds']}",
                "total_time_s": r["total_time_s"],
            }
            for seed in SEEDS:
                d = r["seed_details"][seed]
                row[f"seed_{seed}"] = f"{d['obj']:.4f}({'Y' if d['feas'] else 'N'})"
            w.writerow(row)
    print(f"\nSonuçlar kaydedildi: {path}")


def save_latex_table(all_results, path="results_table.tex"):
    """Doldurulmuş LaTeX tablosu üretir (CPLEX değerleri varsa gap dahil)."""
    lines = []
    lines.append(r"\begin{table}[ht]")
    lines.append(r"\centering")
    lines.append(r"\caption{Computational results: CPLEX vs.\ Genetic Algorithm.}")
    lines.append(r"\label{tab:results}")
    lines.append(r"\begin{tabular}{llrrrrrc}")
    lines.append(r"\hline")
    lines.append(r"\textbf{Group} & \textbf{Inst.} & "
                 r"$z^*_{\text{CPLEX}}$ & $z_{\text{GA}}$ & \textbf{Gap (\%)} & "
                 r"$t_{\text{CPLEX}}$ (s) & $t_{\text{GA}}$ (s) & \textbf{Feas.} \\")
    lines.append(r"\hline")

    group_map = {"S": "Small", "M": "Medium", "L": "Large"}
    prev_group = None
    for r in all_results:
        group = group_map.get(r["name"][0], "")
        prefix = r"\multirow{3}{*}{" + group + "}" if group != prev_group else ""
        prev_group = group

        cplex_str = f"{r['cplex_opt']:.2f}" if r["cplex_opt"] != "" else r"\emph{---}"
        ga_str    = f"{r['ga_best']:.4f}"
        gap_str   = f"{r['ga_gap_best']:.2f}" if r["ga_gap_best"] != "" else r"\emph{---}"
        t_cplex   = r"\emph{---}"   # CPLEX süresi run sonrası elle girilecek
        t_ga      = str(r["total_time_s"])
        feas      = "YES" if r["feas_seeds"] > 0 else "NO"

        lines.append(f"  {prefix} & {r['name']} & {cplex_str} & {ga_str} & "
                     f"{gap_str} & {t_cplex} & {t_ga} & {feas} \\\\")

        if r["name"] in ("S3", "M3", "L3"):
            lines.append(r"\hline")

    lines.append(r"\end{tabular}")
    lines.append(r"\end{table}")

    with open(path, "w") as f:
        f.write("\n".join(lines))
    print(f"LaTeX tablosu kaydedildi: {path}")


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    target = RUN_ONLY if RUN_ONLY else list(INSTANCES.keys())

    print("UAV Pipeline Inspection — Batch Runner")
    print(f"Çalıştırılacak instance'lar: {target}")
    print(f"Her instance için {len(SEEDS)} seed × GA({GA_PARAMS['n_gen']} gen)")

    all_results = []
    total_start = time.time()

    for name in target:
        if name not in INSTANCES:
            print(f"UYARI: '{name}' bulunamadı, atlanıyor.")
            continue
        inst = INSTANCES[name]()
        result = run_instance(name, inst)
        all_results.append(result)

    total_elapsed = time.time() - total_start
    print(f"\n{'='*60}")
    print(f"  Tüm instance'lar tamamlandı — {total_elapsed/60:.1f} dakika")
    print(f"{'='*60}")

    save_csv(all_results, "results.csv")
    save_latex_table(all_results, "results_table.tex")

    # Özet ekrana
    print(f"\n{'Instance':>8}  {'GA Best':>10}  {'Gap%':>8}  {'Feas':>8}  {'Time':>8}")
    print("-"*50)
    for r in all_results:
        gap_str = f"{r['ga_gap_best']:.2f}%" if r["ga_gap_best"] != "" else "  N/A"
        print(f"{r['name']:>8}  {r['ga_best']:>10.4f}  {gap_str:>8}"
              f"  {r['feas_seeds']}/{r['total_seeds']:>4}  {r['total_time_s']:>6.1f}s")
