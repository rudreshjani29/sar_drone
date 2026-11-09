"""Phase 5B: build the walker POA, compare to 5A, run an independent-truth test."""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "phase4-search-area-layout"))
from search_area import example_scenario, rasterize  # noqa: E402
from poa_model import build_prior  # noqa: E402
from walker import (load_walker_params, positions_to_poa, sample_true_locations,  # noqa: E402
                    simulate, _cell_index)


def main():
    grid = rasterize(example_scenario(), cell_m=25.0)
    wp = load_walker_params()

    poa_a = build_prior(grid)
    poa_b = positions_to_poa(grid, simulate(grid, wp, seed=0), wp["smoothing_cells"])

    # ---- sanity ----------------------------------------------------------
    for name, m in (("A", poa_a), ("B", poa_b)):
        assert abs(m.sum() - 1) < 1e-9 and m[~grid.searchable].sum() == 0, name
    print("Sanity checks passed.")

    s = grid.searchable
    a, b = poa_a[s], poa_b[s]
    corr = np.corrcoef(a, b)[0, 1]
    overlap = np.minimum(a, b).sum()
    print(f"5A vs 5B: correlation {corr:.2f}, distribution overlap {overlap:.2f} (1.0 = identical)")

    # ---- independent truth: different seed AND different behaviour ---------
    truth_params = {**wp, "turn_sd_deg": 50, "speed_mps": 0.8, "trail_attract_prob": 0.2,
                    "trail_follow_prob": 0.5}
    truth = sample_true_locations(grid, 3000, seed=123, params=truth_params)
    tix, tiy, ok = _cell_index(grid, truth[:, 0], truth[:, 1])
    truth_cells = tiy[ok] * grid.inside.shape[1] + tix[ok]

    idx = np.flatnonzero(s.ravel())
    rng = np.random.default_rng(0)
    orders = {
        "Uniform (arbitrary order)": rng.permutation(idx),
        "Ranked by 5A (rings + trails)": idx[np.argsort(-poa_a.ravel()[idx])],
        "Ranked by 5B (walker)": idx[np.argsort(-poa_b.ravel()[idx])],
    }
    x = np.arange(1, len(idx) + 1) / len(idx)
    curves = {}
    for name, order in orders.items():
        rank = np.empty(grid.inside.size, dtype=int)
        rank[order] = np.arange(len(order))
        r = rank[truth_cells]
        r = r[np.isin(truth_cells, idx)]
        curves[name] = np.array([(r < k).mean() for k in range(1, len(idx) + 1)])
    k = int(0.2 * len(idx))
    print("\nShare of independent 'true' locations found after covering 20% of area:")
    for name, c in curves.items():
        print(f"  {name:32s} {c[k]*100:5.1f}%")

    fig, ax = plt.subplots(1, 3, figsize=(18, 5.5))
    ext = [grid.xs[0] - 12.5, grid.xs[-1] + 12.5, grid.ys[0] - 12.5, grid.ys[-1] + 12.5]
    for axis, m, t in ((ax[0], poa_a, "5A: rings + trails"), (ax[1], poa_b, "5B: walker simulation")):
        im = axis.imshow(np.where(grid.searchable, m, np.nan), origin="lower", extent=ext, cmap="viridis")
        for tr in grid.trails_xy:
            axis.plot(tr[:, 0], tr[:, 1], "w--", lw=1)
        axis.plot(0, 0, "r*", ms=14, mec="k"); axis.set_aspect("equal"); axis.set_title(t)
        fig.colorbar(im, ax=axis, fraction=0.046)
    for name, c in curves.items():
        ax[2].plot(x * 100, c * 100, label=name)
    ax[2].set_xlabel("% of area covered"); ax[2].set_ylabel("% of true locations found")
    ax[2].set_title("Independent truth (different walker settings)")
    ax[2].legend(); ax[2].grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(HERE, "poa_comparison.png"), dpi=110)
    np.save(os.path.join(HERE, "poa_walker.npy"), poa_b)
    print("Saved poa_comparison.png")


if __name__ == "__main__":
    main()
