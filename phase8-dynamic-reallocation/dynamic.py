"""
Phase 8 (part 1) — Dynamic reallocation with a shared, Bayesian-updated belief map.

No fixed partition: whenever a drone finishes a cell, it picks the next cell
from the SHARED belief map (highest belief per second of travel+search),
skipping cells another drone has already claimed. After every search the cell's
belief is multiplied by (1 - POD). If a drone is lost, its claim is released
and the others pick up its work automatically.

Exactness note: a drone only learns anything from a MISS. Visiting a cell that
holds the person and failing to detect looks the same as visiting an empty
cell, so the plan before the first find is identical for every possible
hiding place. We therefore simulate ONE plan and score it against many
possible hiding places afterwards (no per-truth re-simulation needed).
"""
import heapq
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "phase6-multi-drone-allocation"))
sys.path.insert(0, os.path.join(HERE, "..", "common"))
from allocation import POD, SWAP_TIME_S  # noqa: E402
from drone_config import CRUISE_SPEED_MPS, PLANNABLE_FLIGHT_TIME_MIN  # noqa: E402


def simulate_dynamic(cm, base, n_drones, horizon_s, bayes=True, pod=POD, fail_at=None,
                     speed=CRUISE_SPEED_MPS, budget_s=PLANNABLE_FLIGHT_TIME_MIN * 60,
                     swap_s=SWAP_TIME_S):
    """
    bayes=True : belief *= (1-pod) after each visit (revisits allowed)
    bayes=False: visited cells are removed (one look per cell, like Phase 6)
    fail_at    : list of failure times in seconds per drone (None = never)
    Returns visits: list of (time_s, cell, drone).
    """
    fail_at = fail_at or [np.inf] * n_drones
    t_cell = cm.cell_m / speed
    belief = cm.mass.copy()
    claimed = np.zeros(cm.n, dtype=bool)
    d_home = np.hypot(cm.x - base[0], cm.y - base[1])

    pos = [np.array(base, float) for _ in range(n_drones)]
    used = [0.0] * n_drones
    target = [None] * n_drones
    heap = [(0.0, d) for d in range(n_drones)]
    heapq.heapify(heap)
    visits = []

    while heap:
        t, d = heapq.heappop(heap)
        if t > horizon_s:
            break
        if t >= fail_at[d]:                       # drone lost: release its claim
            if target[d] is not None:
                claimed[target[d]] = False
                target[d] = None
            continue

        c = target[d]
        if c is not None:                         # arrived and searched cell c
            visits.append((t, c, d))
            pos[d] = np.array([cm.x[c], cm.y[c]])
            claimed[c] = False
            target[d] = None
            belief[c] = belief[c] * (1 - pod) if bayes else 0.0

        dist = np.hypot(cm.x - pos[d][0], cm.y - pos[d][1])
        step = dist / speed + t_cell
        feasible = (~claimed) & (belief > 0) & (used[d] + step + d_home / speed <= budget_s)
        if feasible.any():
            score = np.where(feasible, belief / step, -1.0)
            k = int(np.argmax(score))
            claimed[k] = True
            target[d] = k
            used[d] += step[k]
            heapq.heappush(heap, (t + step[k], d))
        else:
            if np.allclose(pos[d], base) and used[d] == 0.0:
                continue                           # nothing reachable even on a fresh battery
            back = np.hypot(*(pos[d] - base)) / speed + swap_s
            pos[d] = np.array(base, float)
            used[d] = 0.0
            heapq.heappush(heap, (t + back, d))
    return visits
