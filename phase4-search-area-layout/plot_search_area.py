"""Render the Phase 4 search-area layout to PNG and export it as JSON."""

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np

from search_area import example_scenario, rasterize

HERE = os.path.dirname(os.path.abspath(__file__))


def main(cell_m=25.0):
    area = example_scenario()
    grid = rasterize(area, cell_m=cell_m)
    info = grid.summary()

    # category map: 0 outside, 1 searchable, 2 trail, 3 water
    cat = np.zeros(grid.inside.shape, dtype=int)
    cat[grid.searchable] = 1
    cat[grid.trail & grid.searchable] = 2
    cat[grid.water & grid.inside] = 3
    cmap = ListedColormap(["#f2f2f2", "#cfe8c4", "#c9a66b", "#7fb2e5"])

    fig, ax = plt.subplots(figsize=(9, 7))
    extent = [grid.xs[0] - cell_m / 2, grid.xs[-1] + cell_m / 2,
              grid.ys[0] - cell_m / 2, grid.ys[-1] + cell_m / 2]
    ax.imshow(cat, origin="lower", extent=extent, cmap=cmap, vmin=0, vmax=3)

    b = np.vstack([grid.boundary_xy, grid.boundary_xy[0]])
    ax.plot(b[:, 0], b[:, 1], "k-", lw=2, label="Search boundary")
    for i, t in enumerate(grid.trails_xy):
        ax.plot(t[:, 0], t[:, 1], color="#7a5230", lw=1.5, ls="--",
                label="Trail" if i == 0 else None)
    ax.plot(*grid.pls_xy, marker="*", ms=18, color="red", mec="k",
            ls="none", label="Point Last Seen")

    ax.set_aspect("equal")
    ax.set_xlabel("East of PLS (m)")
    ax.set_ylabel("North of PLS (m)")
    ax.set_title(f"{area.name}\n{info['area_km2']:.2f} km² total, "
                 f"{info['searchable_km2']:.2f} km² searchable "
                 f"({cell_m:.0f} m cells), subject: {area.subject_category}")
    ax.legend(loc="lower left")
    out_png = os.path.join(HERE, "search_area_layout.png")
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)

    area.to_json(os.path.join(HERE, "example_scenario.json"))
    print("Grid summary:", info)
    print("Saved", out_png)


if __name__ == "__main__":
    main()
