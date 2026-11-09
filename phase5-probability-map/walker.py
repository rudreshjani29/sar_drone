"""
Phase 5B — Agent-based lost-person walker.

Simulates many lost hikers starting near the PLS. Each takes a correlated
random walk, is drawn toward nearby trails and follows them, cannot enter
water or leave the boundary (steps are rejected), and may stop (rest) at
random. Final positions form an independent probability map (POA_B) and a
sampler for "where the hidden person actually is" in later evaluations.

EVERYTHING here is an assumed behaviour model — see walker_params.json.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "phase4-search-area-layout"))
from search_area import SearchGrid  # noqa: E402


def load_walker_params(path=None):
    with open(path or os.path.join(HERE, "walker_params.json")) as f:
        return json.load(f)


def _trail_field(grid: SearchGrid):
    """For each cell centre: distance to nearest trail, unit vector to it, trail tangent."""
    X, Y = np.meshgrid(grid.xs, grid.ys)
    best = np.full(X.shape, np.inf)
    vx = np.zeros(X.shape); vy = np.zeros(X.shape)
    tx = np.zeros(X.shape); ty = np.zeros(X.shape)
    for t in grid.trails_xy:
        for (x1, y1), (x2, y2) in zip(t[:-1], t[1:]):
            dx, dy = x2 - x1, y2 - y1
            seg2 = dx * dx + dy * dy
            u = np.clip(((X - x1) * dx + (Y - y1) * dy) / seg2, 0, 1)
            px, py = x1 + u * dx, y1 + u * dy
            d = np.hypot(X - px, Y - py)
            closer = d < best
            best = np.where(closer, d, best)
            n = np.maximum(d, 1e-9)
            vx = np.where(closer, (px - X) / n, vx)
            vy = np.where(closer, (py - Y) / n, vy)
            L = np.sqrt(seg2)
            tx = np.where(closer, dx / L, tx)
            ty = np.where(closer, dy / L, ty)
    return best, vx, vy, tx, ty


def _cell_index(grid, x, y):
    cs = grid.cell_m
    ix = np.floor((x - (grid.xs[0] - cs / 2)) / cs).astype(int)
    iy = np.floor((y - (grid.ys[0] - cs / 2)) / cs).astype(int)
    ny, nx = grid.inside.shape
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    return np.clip(ix, 0, nx - 1), np.clip(iy, 0, ny - 1), ok


def simulate(grid: SearchGrid, params=None, seed=0):
    """Return final (x, y) positions of all agents, shape (n, 2)."""
    p = params or load_walker_params()
    rng = np.random.default_rng(seed)
    n = p["n_agents"]
    dist, vx, vy, tx, ty = _trail_field(grid)
    walkable = grid.searchable

    x = rng.normal(grid.pls_xy[0], p["start_sigma_m"], n)
    y = rng.normal(grid.pls_xy[1], p["start_sigma_m"], n)
    heading = rng.uniform(0, 2 * np.pi, n)
    speed = np.clip(rng.normal(p["speed_mps"], p["speed_sd_mps"], n), 0.05, None)
    active = np.ones(n, dtype=bool)
    turn_sd = np.radians(p["turn_sd_deg"])

    steps = int(p["duration_h"] * 3600 / p["dt_s"])
    for _ in range(steps):
        ix, iy, _ok = _cell_index(grid, x, y)
        d = dist[iy, ix]

        heading = heading + rng.normal(0, turn_sd, n)

        near = d < p["trail_attract_radius_m"]
        on = d < p["on_trail_radius_m"]
        # follow trail: align with tangent (sign chosen to match current heading)
        follow = on & (rng.random(n) < p["trail_follow_prob"])
        ttx, tty = tx[iy, ix], ty[iy, ix]
        sign = np.sign(np.cos(heading) * ttx + np.sin(heading) * tty)
        sign[sign == 0] = 1
        heading = np.where(follow, np.arctan2(sign * tty, sign * ttx), heading)
        # steer toward trail when near but not on it
        steer = near & ~on & (rng.random(n) < p["trail_attract_prob"])
        heading = np.where(steer, np.arctan2(vy[iy, ix], vx[iy, ix]), heading)

        nx_ = x + np.cos(heading) * speed * p["dt_s"]
        ny_ = y + np.sin(heading) * speed * p["dt_s"]
        jx, jy, ok = _cell_index(grid, nx_, ny_)
        allowed = ok & walkable[jy, jx] & active
        x = np.where(allowed, nx_, x)
        y = np.where(allowed, ny_, y)
        # blocked agents pick a new random heading (bounce)
        blocked = active & ~allowed
        heading = np.where(blocked, rng.uniform(0, 2 * np.pi, n), heading)

        active &= rng.random(n) >= p["stop_prob_per_step"]

    return np.column_stack([x, y])


def positions_to_poa(grid: SearchGrid, pos, smoothing_cells=1.0):
    ix, iy, ok = _cell_index(grid, pos[:, 0], pos[:, 1])
    H = np.zeros(grid.inside.shape)
    np.add.at(H, (iy[ok], ix[ok]), 1.0)
    if smoothing_cells > 0:
        r = int(np.ceil(3 * smoothing_cells))
        k = np.exp(-0.5 * (np.arange(-r, r + 1) / smoothing_cells) ** 2)
        k /= k.sum()
        H = np.apply_along_axis(lambda m: np.convolve(m, k, mode="same"), 1, H)
        H = np.apply_along_axis(lambda m: np.convolve(m, k, mode="same"), 0, H)
    H = H * grid.searchable
    return H / H.sum()


def sample_true_locations(grid, n, seed=1, params=None):
    """Independent 'ground truth' positions (different seed than any map built)."""
    return simulate(grid, {**(params or load_walker_params()), "n_agents": n}, seed=seed)
