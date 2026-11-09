"""
Phase 5A — Probability-of-Area (POA) map, ring/distance model + terrain modifiers.

Layers:
  1. Distance prior around the PLS (Rayleigh 2-D, fitted to one percentile anchor)
  2. Trail attraction multiplier (Phase 4 trail geometry)
  3. Bayesian update after searching cells (probability of detection, POD)

All parameters live in lpb_params.json and are ILLUSTRATIVE until replaced
with the real ISRID hiker table. See README.
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "phase4-search-area-layout"))
from search_area import SearchGrid  # noqa: E402


def load_params(path=None):
    with open(path or os.path.join(HERE, "lpb_params.json")) as f:
        return json.load(f)


def rayleigh_sigma_m(percentile, distance_km):
    """Sigma such that P(r <= distance) = percentile for a 2-D isotropic Gaussian."""
    return distance_km * 1000.0 / math.sqrt(-2.0 * math.log(1.0 - percentile))


def _trail_proximity(grid: SearchGrid, sigma_m):
    X, Y = np.meshgrid(grid.xs, grid.ys)
    best = np.full(X.shape, np.inf)
    for t in grid.trails_xy:
        for (x1, y1), (x2, y2) in zip(t[:-1], t[1:]):
            dx, dy = x2 - x1, y2 - y1
            seg2 = dx * dx + dy * dy
            u = np.zeros_like(X) if seg2 == 0 else np.clip(
                ((X - x1) * dx + (Y - y1) * dy) / seg2, 0, 1)
            best = np.minimum(best, np.hypot(X - (x1 + u * dx), Y - (y1 + u * dy)))
    if not np.isfinite(best).any():
        return np.zeros(X.shape)
    return np.exp(-0.5 * (best / sigma_m) ** 2)


def build_prior(grid: SearchGrid, params=None):
    """Return a POA grid (sums to 1 over searchable cells)."""
    p = params or load_params()
    dm, tm, wm = p["distance_model"], p["trail_model"], p["water_model"]

    sigma = rayleigh_sigma_m(dm["anchor_percentile"], dm["anchor_distance_km"])
    X, Y = np.meshgrid(grid.xs, grid.ys)
    px, py = grid.pls_xy
    r2 = (X - px) ** 2 + (Y - py) ** 2
    density = np.exp(-r2 / (2 * sigma ** 2))                 # per-area density

    density = density * (1.0 + tm["weight"] * _trail_proximity(grid, tm["corridor_sigma_m"]))

    mask = grid.inside.astype(float)
    mask[grid.water] = wm["probability_inside_water"]
    poa = density * mask
    total = poa.sum()
    if total <= 0:
        raise ValueError("Prior has no mass inside the search area")
    return poa / total


def bayes_update(poa, searched_mask, pod):
    """
    Posterior after searching `searched_mask` cells with detection probability
    `pod` (scalar or array) and finding nothing.
    Returns (new_poa, probability_of_success_this_search).
    """
    pod = np.broadcast_to(pod, poa.shape)
    found = float((poa * pod * searched_mask).sum())
    post = poa * np.where(searched_mask, 1.0 - pod, 1.0)
    return post / post.sum(), found


def capture_curve(poa, order):
    """Cumulative POA captured as cells are searched in the given order (flat indices)."""
    flat = poa.ravel()
    return np.cumsum(flat[order]) / flat.sum()
