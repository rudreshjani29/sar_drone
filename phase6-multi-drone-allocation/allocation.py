"""
Phase 6 — Multi-drone search allocation (pure Python simulation, no PX4).

Splits the probability map between N drones and simulates each drone's
battery-limited flight, producing a visit time for every cell. Static plan:
the map is NOT updated during the flight (that is Phase 8).

Simplifications (all listed in README):
  * Grid is coarsened to 50 m cells; visiting a cell covers it with one detection
    chance (POD). Real camera footprint/overlap is not modelled.
  * Drones fly straight lines at constant speed; no wind, climb or turn time.
  * All drones launch from a common base at the PLS; battery swap takes fixed time.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "common"))
from drone_config import CRUISE_SPEED_MPS, PLANNABLE_FLIGHT_TIME_MIN  # noqa: E402

SWAP_TIME_S = 300.0        # ASSUMED battery swap time at base
POD = 0.8                  # ASSUMED detection chance per visited cell (Phase 3.1 saw ~0.86 at altitude)
COARSEN = 2                # 25 m cells -> 50 m cells


class CoarseMap:
    """Coarsened POA map as flat arrays of searchable cells."""

    def __init__(self, grid, poa, factor=COARSEN):
        ny, nx = poa.shape
        py, px = (-ny) % factor, (-nx) % factor
        pad = lambda a: np.pad(a, ((0, py), (0, px)))
        P = pad(poa)
        S = pad(grid.searchable.astype(float))
        X = np.pad(np.tile(grid.xs, (ny, 1)), ((0, py), (0, px)), mode="edge")
        Y = np.pad(np.tile(grid.ys[:, None], (1, nx)), ((0, py), (0, px)), mode="edge")
        f = factor
        blk = lambda a: a.reshape(a.shape[0] // f, f, a.shape[1] // f, f)
        mass = blk(P).sum(axis=(1, 3))
        srch = blk(S).sum(axis=(1, 3)) > 0
        cx = blk(X).mean(axis=(1, 3))
        cy = blk(Y).mean(axis=(1, 3))
        self.shape = mass.shape
        self.factor = f
        keep = srch & (mass > 0)
        self.id_grid = -np.ones(mass.shape, dtype=int)
        self.rows, self.cols = np.nonzero(keep)
        self.id_grid[keep] = np.arange(len(self.rows))
        self.x, self.y = cx[keep], cy[keep]
        self.mass = mass[keep]
        self.mass = self.mass / self.mass.sum()
        self.n = len(self.mass)
        self.cell_m = grid.cell_m * f

    def fine_to_cell(self, fine_ix, fine_iy):
        return self.id_grid[fine_iy // self.factor, fine_ix // self.factor]


# ---------------------------------------------------------------------------
# Assignment strategies
# ---------------------------------------------------------------------------
def split_equal_area_strips(cm: CoarseMap, n):
    order = np.argsort(cm.x, kind="stable")
    return [g for g in np.array_split(order, n)]


def split_probability_sectors(cm: CoarseMap, base, n):
    """Pie slices around the base, each holding ~equal probability mass."""
    ang = np.arctan2(cm.y - base[1], cm.x - base[0])
    order = np.argsort(ang, kind="stable")
    cum = np.cumsum(cm.mass[order])
    cuts = np.searchsorted(cum, np.arange(1, n) / n * cum[-1])
    return np.split(order, cuts)


# ---------------------------------------------------------------------------
# One drone's flight
# ---------------------------------------------------------------------------
def fly_drone(cm: CoarseMap, cells, mode, base, speed=CRUISE_SPEED_MPS,
              budget_s=PLANNABLE_FLIGHT_TIME_MIN * 60, swap_s=SWAP_TIME_S):
    """
    mode: 'sweep'  (snake order by column, uniform-search style) or
          'greedy' (next cell = highest mass per second of travel+search).
    Returns (visit_time dict {cell: seconds}, finish_time_s, path list of (t, x, y, sortie)).
    """
    t_cell = cm.cell_m / speed
    remaining = np.ones(len(cells), dtype=bool)
    cx, cy, mass = cm.x[cells], cm.y[cells], cm.mass[cells]

    if mode == "sweep":
        col, row = cm.cols[cells], cm.rows[cells]
        key_row = np.where(col % 2 == 0, row, -row)
        sweep_order = list(np.lexsort((key_row, col)))
    visit, path = {}, []
    pos = np.array(base, float)
    t, used, sortie = 0.0, 0.0, 1
    path.append((0.0, pos[0], pos[1], sortie))
    ptr = 0

    while remaining.any():
        if mode == "sweep":
            while not remaining[sweep_order[ptr]]:
                ptr += 1
            k = sweep_order[ptr]
        else:
            idx = np.flatnonzero(remaining)
            d = np.hypot(cx[idx] - pos[0], cy[idx] - pos[1])
            k = idx[np.argmax(mass[idx] / (d / speed + t_cell))]

        d_move = np.hypot(cx[k] - pos[0], cy[k] - pos[1])
        d_home = np.hypot(cx[k] - base[0], cy[k] - base[1])
        need = d_move / speed + t_cell + d_home / speed
        if used + need > budget_s:
            at_base = np.allclose(pos, base)
            if at_base and need > budget_s:      # unreachable even on a fresh battery
                remaining[k] = False
                continue
            t += np.hypot(*(pos - base)) / speed + swap_s
            pos = np.array(base, float)
            used, sortie = 0.0, sortie + 1
            path.append((t, pos[0], pos[1], sortie))
            continue
        t += d_move / speed + t_cell
        used += d_move / speed + t_cell
        pos = np.array([cx[k], cy[k]])
        visit[cells[k]] = t
        remaining[k] = False
        path.append((t, pos[0], pos[1], sortie))
    return visit, t, path


def run_strategy(cm, base, groups, mode):
    visit_time = np.full(cm.n, np.inf)
    finish, paths = 0.0, []
    for g in groups:
        v, tf, path = fly_drone(cm, g, mode, base)
        for c, tv in v.items():
            visit_time[c] = tv
        finish = max(finish, tf)
        paths.append(path)
    return visit_time, finish, paths
