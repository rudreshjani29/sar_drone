"""Build the Phase 5A POA map for the Phase 4 example, plot it, run sanity checks."""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "phase4-search-area-layout"))
from search_area import example_scenario, rasterize  # noqa: E402
from poa_model import bayes_update, build_prior, capture_curve, load_params, rayleigh_sigma_m  # noqa: E402


def main():
    params = load_params()
    area = example_scenario()
    grid = rasterize(area, cell_m=25.0)
    poa = build_prior(grid, params)

    # ---- sanity checks ----------------------------------------------------
    assert abs(poa.sum() - 1.0) < 1e-9, "POA must sum to 1"
    assert poa[~grid.inside].sum() == 0, "no mass outside boundary"
    assert poa[grid.water].sum() == 0, "no mass in water"
    pod = 0.8
    top = poa >= np.quantile(poa[grid.searchable], 0.9)
    post, found = bayes_update(poa, top, pod)
    assert abs(post.sum() - 1.0) < 1e-9
    assert post[top].sum() < poa[top].sum(), "searched cells must lose probability"
    print("Sanity checks passed.")
    sigma = rayleigh_sigma_m(params["distance_model"]["anchor_percentile"],
                             params["distance_model"]["anchor_distance_km"])
    print(f"Rayleigh sigma = {sigma:.0f} m; "
          f"mass within 300 m of PLS = {poa[np.hypot(*np.meshgrid(grid.xs, grid.ys)) < 300].sum():.2f}")
    print(f"Searching top-10% cells at POD {pod}: success chance this pass = {found:.2f}")

    # ---- capture curves: POA-ranked vs uniform (random) order ---------------
    idx = np.flatnonzero(grid.searchable.ravel())
    ranked = idx[np.argsort(-poa.ravel()[idx])]
    rng = np.random.default_rng(0)
    rand = rng.permutation(idx)
    x = np.arange(1, len(idx) + 1) / len(idx)
    c_rank, c_rand = capture_curve(poa, ranked), capture_curve(poa, rand)

    fig, ax = plt.subplots(1, 2, figsize=(14, 6))
    ext = [grid.xs[0] - 12.5, grid.xs[-1] + 12.5, grid.ys[0] - 12.5, grid.ys[-1] + 12.5]
    im = ax[0].imshow(np.where(grid.searchable, poa, np.nan), origin="lower",
                      extent=ext, cmap="viridis")
    b = np.vstack([grid.boundary_xy, grid.boundary_xy[0]])
    ax[0].plot(b[:, 0], b[:, 1], "k-", lw=1.5)
    for t in grid.trails_xy:
        ax[0].plot(t[:, 0], t[:, 1], "w--", lw=1)
    ax[0].plot(0, 0, "r*", ms=16, mec="k")
    ax[0].set_aspect("equal")
    ax[0].set_title("POA map (ILLUSTRATIVE parameters)")
    ax[0].set_xlabel("East of PLS (m)"); ax[0].set_ylabel("North of PLS (m)")
    fig.colorbar(im, ax=ax[0], label="probability per cell")

    ax[1].plot(x * 100, c_rank * 100, label="Search highest-POA cells first")
    ax[1].plot(x * 100, c_rand * 100, label="Uniform / arbitrary order")
    ax[1].set_xlabel("% of searchable area covered")
    ax[1].set_ylabel("% of probability captured")
    ax[1].set_title("Prior-value curve (assumes the model is right)")
    ax[1].legend(); ax[1].grid(alpha=0.3)
    fig.tight_layout()
    out = os.path.join(HERE, "poa_map.png")
    fig.savefig(out, dpi=120)
    np.save(os.path.join(HERE, "poa_prior.npy"), poa)
    k = np.searchsorted(x, 0.2)
    print(f"Covering 20% of area: POA-ranked captures {c_rank[k]*100:.0f}% vs {c_rand[k]*100:.0f}% arbitrary")
    print("Saved", out)


if __name__ == "__main__":
    main()
