"""Phase 6: compare single-drone and multi-drone allocation strategies."""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for sub in ("phase4-search-area-layout", "phase5-probability-map"):
    sys.path.insert(0, os.path.join(HERE, "..", sub))
from search_area import example_scenario, rasterize  # noqa: E402
from poa_model import build_prior  # noqa: E402
from walker import load_walker_params, sample_true_locations, _cell_index  # noqa: E402
from allocation import (POD, CoarseMap, run_strategy, split_equal_area_strips,  # noqa: E402
                        split_probability_sectors)

BASE = (0.0, 0.0)  # PLS


def metrics(visit_time, truth_cells):
    t = visit_time[truth_cells]
    reached = np.isfinite(t)
    out = {"covered%": reached.mean() * 100}
    for q in (0.5, 0.8):
        out[f"t{int(q*100)}_min"] = (np.quantile(t, q) / 60) if (reached.mean() >= q) else np.inf
    for T in (30, 60, 120):
        out[f"found<{T}m%"] = POD * (t <= T * 60).mean() * 100
    return out


def main():
    grid = rasterize(example_scenario(), cell_m=25.0)
    poa = build_prior(grid)                     # plan with the 5A map
    cm = CoarseMap(grid, poa)

    # independent truth: different walker settings and seed (as in Phase 5B)
    wp = load_walker_params()
    tp = {**wp, "turn_sd_deg": 50, "speed_mps": 0.8, "trail_attract_prob": 0.2, "trail_follow_prob": 0.5}
    truth = sample_true_locations(grid, 3000, seed=123, params=tp)
    ix, iy, ok = _cell_index(grid, truth[:, 0], truth[:, 1])
    tc = cm.fine_to_cell(ix[ok], iy[ok])
    tc = tc[tc >= 0]

    all_cells = np.arange(cm.n)
    results, finish, store = {}, {}, {}

    def add(name, groups, mode):
        v, f, paths = run_strategy(cm, BASE, groups, mode)
        results[name] = metrics(v, tc); finish[name] = f / 60; store[name] = (v, paths, groups)

    add("1 drone, sweep", [all_cells], "sweep")
    add("1 drone, POA-greedy", [all_cells], "greedy")
    for n in (3, 4, 5):
        add(f"{n} drones, equal area + sweep", split_equal_area_strips(cm, n), "sweep")
        add(f"{n} drones, equal area + greedy", split_equal_area_strips(cm, n), "greedy")
        add(f"{n} drones, POA sectors + greedy", split_probability_sectors(cm, BASE, n), "greedy")

    print(f"Coarse cells: {cm.n} ({cm.cell_m:.0f} m), truth samples: {len(tc)}, POD={POD}")
    print(f"{'strategy':36s} {'50% (min)':>9s} {'80% (min)':>9s} {'found<30m':>9s} {'<60m':>6s} {'<120m':>6s} {'done (min)':>10s}")
    for name, r in results.items():
        print(f"{name:36s} {r['t50_min']:9.1f} {r['t80_min']:9.1f} {r['found<30m%']:8.0f}% "
              f"{r['found<60m%']:5.0f}% {r['found<120m%']:5.0f}% {finish[name]:10.1f}")

    # ---------- figure ----------
    fig, ax = plt.subplots(1, 3, figsize=(19, 5.8))
    show = ["1 drone, sweep", "1 drone, POA-greedy", "4 drones, equal area + sweep",
            "4 drones, equal area + greedy", "4 drones, POA sectors + greedy"]
    tmax = 240
    ts = np.arange(0, tmax * 60, 30)
    for name in show:
        v = store[name][0][tc]
        ax[0].plot(ts / 60, [POD * (v <= T).mean() * 100 for T in ts], label=name)
    ax[0].set_xlabel("time since launch (min)"); ax[0].set_ylabel("% of hidden people found")
    ax[0].set_title("Time to find (independent truth, POD 0.8)"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)

    ns = [3, 4, 5]
    for label in ("equal area + sweep", "equal area + greedy", "POA sectors + greedy"):
        ax[1].plot(ns, [results[f"{n} drones, {label}"]["t50_min"] for n in ns], marker="o", label=label)
    for name, ls in (("1 drone, sweep", ":"), ("1 drone, POA-greedy", "--")):
        ax[1].axhline(results[name]["t50_min"], ls=ls, color="gray", label=name)
    ax[1].set_xticks(ns); ax[1].set_xlabel("number of drones"); ax[1].set_ylabel("time to reach 50% of locations (min)")
    ax[1].set_title("Effect of drone count and strategy"); ax[1].grid(alpha=0.3); ax[1].legend(fontsize=8)

    v, paths, groups = store["4 drones, POA sectors + greedy"]
    colors = plt.cm.tab10(np.arange(4))
    for g, col in zip(groups, colors):
        ax[2].scatter(cm.x[g], cm.y[g], s=14, color=col, alpha=0.5)
    for path, col in zip(paths, colors):
        p = np.array([(x, y) for _, x, y, _ in path])
        ax[2].plot(p[:, 0], p[:, 1], color=col, lw=0.4, alpha=0.6)
    for tr in grid.trails_xy:
        ax[2].plot(tr[:, 0], tr[:, 1], "k--", lw=1)
    ax[2].plot(*BASE, "r*", ms=15, mec="k"); ax[2].set_aspect("equal")
    ax[2].set_title("4 drones, probability sectors + greedy routes")
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "allocation_comparison.png"), dpi=110)
    print("Saved allocation_comparison.png")


if __name__ == "__main__":
    main()
