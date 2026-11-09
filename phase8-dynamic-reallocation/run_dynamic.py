"""Phase 8 (part 1): dynamic reallocation vs static plans, incl. a mid-mission drone failure."""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for sub in ("phase4-search-area-layout", "phase5-probability-map", "phase6-multi-drone-allocation"):
    sys.path.insert(0, os.path.join(HERE, "..", sub))
from search_area import example_scenario, rasterize  # noqa: E402
from poa_model import build_prior  # noqa: E402
from walker import load_walker_params, sample_true_locations, _cell_index  # noqa: E402
from allocation import (POD, CoarseMap, fly_drone, split_equal_area_strips,  # noqa: E402
                        split_probability_sectors)
from dynamic import simulate_dynamic  # noqa: E402

BASE = (0.0, 0.0)
HORIZON_S = 240 * 60
DRAWS = 10


def static_visits(cm, groups, mode, fail_at=None):
    fail_at = fail_at or [np.inf] * len(groups)
    out = []
    for d, g in enumerate(groups):
        v, _, _ = fly_drone(cm, g, mode, BASE)
        out += [(t, c, d) for c, t in v.items() if t < fail_at[d] and t <= HORIZON_S]
    return out


def found_times(visits, truth_cells, rng):
    """Time at which each (truth, detection-draw) is found; inf if never within horizon."""
    by_cell = {}
    for t, c, _ in sorted(visits):
        by_cell.setdefault(c, []).append(t)
    res = []
    for c in truth_cells:
        ts = by_cell.get(c, [])
        for _ in range(DRAWS):
            hit = np.inf
            for t in ts:
                if rng.random() < POD:
                    hit = t
                    break
            res.append(hit)
    return np.array(res)


def summarize(ft, visits, cm):
    s = {f"<{T}m": (ft <= T * 60).mean() * 100 for T in (30, 60, 120, 240)}
    s["t50"] = np.quantile(ft, 0.5) / 60 if (ft < np.inf).mean() >= 0.5 else np.inf
    first = np.full(cm.n, np.inf)
    for t, c, _ in visits:
        first[c] = min(first[c], t)
    s["cover_all"] = first.max() / 60
    s["visits"] = len(visits)
    return s


def main():
    grid = rasterize(example_scenario(), cell_m=25.0)
    cm = CoarseMap(grid, build_prior(grid))

    wp = load_walker_params()
    tp = {**wp, "turn_sd_deg": 50, "speed_mps": 0.8, "trail_attract_prob": 0.2, "trail_follow_prob": 0.5}
    truth = sample_true_locations(grid, 3000, seed=123, params=tp)
    ix, iy, ok = _cell_index(grid, truth[:, 0], truth[:, 1])
    tc = cm.fine_to_cell(ix[ok], iy[ok]); tc = tc[tc >= 0]

    def evaluate(n, fail=None):
        fa = [np.inf] * n
        if fail:
            fa[fail[0]] = fail[1] * 60
        plans = {
            "static: equal area + greedy": static_visits(cm, split_equal_area_strips(cm, n), "greedy", fa),
            "static: POA sectors + greedy": static_visits(cm, split_probability_sectors(cm, BASE, n), "greedy", fa),
            "dynamic, one look per cell": simulate_dynamic(cm, BASE, n, HORIZON_S, bayes=False, fail_at=fa),
            "dynamic + Bayesian revisits": simulate_dynamic(cm, BASE, n, HORIZON_S, bayes=True, fail_at=fa),
        }
        out = {}
        for name, v in plans.items():
            ft = found_times(v, tc, np.random.default_rng(7))
            out[name] = (ft, summarize(ft, v, cm))
        return out

    def table(title, res):
        print(f"\n{title}")
        print(f"{'strategy':32s} {'t50 (min)':>9s} {'<30m':>6s} {'<60m':>6s} {'<120m':>6s} {'<240m':>6s} {'all cells seen (min)':>20s}")
        for name, (_, s) in res.items():
            cov = f"{s['cover_all']:.0f}" if np.isfinite(s["cover_all"]) else ">240"
            print(f"{name:32s} {s['t50']:9.1f} {s['<30m']:5.0f}% {s['<60m']:5.0f}% {s['<120m']:5.0f}% {s['<240m']:5.0f}% {cov:>20s}")

    normal = evaluate(4)
    table("4 drones, no failures", normal)
    failure = evaluate(4, fail=(1, 20))
    table("4 drones, drone #2 lost at t = 20 min", failure)

    # Extra check: truth drawn FROM the planning map itself (model is exactly right)
    rng = np.random.default_rng(3)
    tc_matched = rng.choice(cm.n, size=3000, p=cm.mass)
    print("\nMatched-model check (hidden people drawn from the planning map), 4 drones:")
    for name, bayes in (("dynamic, one look per cell", False), ("dynamic + Bayesian revisits", True)):
        v = simulate_dynamic(cm, BASE, 4, HORIZON_S, bayes=bayes)
        ft = found_times(v, tc_matched, np.random.default_rng(7))
        print(f"  {name:30s} t50={np.quantile(ft, 0.5)/60:5.1f} min  <60m={(ft<=3600).mean()*100:3.0f}%  "
              f"<120m={(ft<=7200).mean()*100:3.0f}%  <240m={(ft<=14400).mean()*100:3.0f}%")

    sweep = {}
    for n in (3, 4, 5):
        r = evaluate(n)
        sweep[n] = {k: v[1]["t50"] for k, v in r.items()}
    print("\nTime to find 50% (min) by drone count:")
    for n, d in sweep.items():
        print(n, {k.split(':')[-1].strip(): round(v, 1) for k, v in d.items()})

    fig, ax = plt.subplots(1, 3, figsize=(19, 5.5))
    ts = np.arange(0, HORIZON_S, 30)
    for a, res, title in ((ax[0], normal, "4 drones, normal mission"),
                          (ax[1], failure, "4 drones, drone #2 lost at 20 min")):
        for name, (ft, _) in res.items():
            a.plot(ts / 60, [(ft <= T).mean() * 100 for T in ts], label=name)
        a.set_title(title); a.set_xlabel("time since launch (min)")
        a.set_ylabel("% of hidden people found"); a.grid(alpha=0.3); a.legend(fontsize=8)
    ns = [3, 4, 5]
    for k in normal:
        ax[2].plot(ns, [sweep[n][k] for n in ns], marker="o", label=k)
    ax[2].set_xticks(ns); ax[2].set_xlabel("number of drones")
    ax[2].set_ylabel("time to find 50% (min)"); ax[2].set_title("Drone count")
    ax[2].grid(alpha=0.3); ax[2].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "dynamic_comparison.png"), dpi=110)
    print("Saved dynamic_comparison.png")


if __name__ == "__main__":
    main()
