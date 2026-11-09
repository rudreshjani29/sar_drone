"""
Phase 4 — Search Area Layout

Defines WHERE the search happens: a boundary polygon, the Point Last Seen
(PLS), a subject-category label, and terrain features (trails, water).
Rasterizes everything onto a regular grid in local metres so Phase 5 (the
probability-of-area map) and Phase 6+ (multi-drone allocation) can work on
plain numpy arrays.

Scope notes (honest limitations):
  * The area is hand-defined here. In the full vision a VTOL survey would
    produce it; that input is not implemented yet.
  * subject_category is only a LABEL. No Lost Person Behavior statistics are
    used or invented here — Phase 5 supplies those.
  * Terrain is limited to trails and water. No elevation / vegetation yet.
"""

import json
import math
from dataclasses import dataclass, field

import numpy as np
from matplotlib.path import Path

EARTH_RADIUS_M = 6371000.0


# ---------------------------------------------------------------------------
# Local projection (equirectangular around an origin; fine for a few km)
# ---------------------------------------------------------------------------
class LocalFrame:
    """East/North metres relative to an origin lat/lon."""

    def __init__(self, origin_lat, origin_lon):
        self.lat0 = origin_lat
        self.lon0 = origin_lon
        self._m_per_deg_lat = math.pi / 180 * EARTH_RADIUS_M
        self._m_per_deg_lon = self._m_per_deg_lat * math.cos(math.radians(origin_lat))

    def to_local(self, lat, lon):
        return ((lon - self.lon0) * self._m_per_deg_lon,
                (lat - self.lat0) * self._m_per_deg_lat)

    def to_latlon(self, x, y):
        return (self.lat0 + y / self._m_per_deg_lat,
                self.lon0 + x / self._m_per_deg_lon)


# ---------------------------------------------------------------------------
# Scenario definition
# ---------------------------------------------------------------------------
@dataclass
class SearchArea:
    name: str
    boundary: list                      # [(lat, lon), ...] polygon, not closed
    pls: tuple                          # (lat, lon) Point Last Seen
    subject_category: str = "unspecified"
    trails: list = field(default_factory=list)   # [[(lat, lon), ...], ...] polylines
    water: list = field(default_factory=list)    # [[(lat, lon), ...], ...] polygons

    def validate(self):
        if len(self.boundary) < 3:
            raise ValueError("boundary needs at least 3 points")
        frame = LocalFrame(*self.pls)
        poly = Path([frame.to_local(*p) for p in self.boundary])
        if not poly.contains_point(frame.to_local(*self.pls)):
            raise ValueError("PLS must lie inside the search boundary")

    def to_json(self, path):
        with open(path, "w") as f:
            json.dump(self.__dict__, f, indent=2)

    @staticmethod
    def from_json(path):
        with open(path) as f:
            d = json.load(f)
        conv = lambda pts: [tuple(p) for p in pts]
        return SearchArea(
            name=d["name"], boundary=conv(d["boundary"]), pls=tuple(d["pls"]),
            subject_category=d.get("subject_category", "unspecified"),
            trails=[conv(t) for t in d.get("trails", [])],
            water=[conv(w) for w in d.get("water", [])],
        )


# ---------------------------------------------------------------------------
# Rasterization
# ---------------------------------------------------------------------------
@dataclass
class SearchGrid:
    frame: LocalFrame
    cell_m: float
    xs: np.ndarray            # cell-centre x (east), shape (nx,)
    ys: np.ndarray            # cell-centre y (north), shape (ny,)
    inside: np.ndarray        # bool (ny, nx): inside boundary
    water: np.ndarray         # bool (ny, nx): water cell
    trail: np.ndarray         # bool (ny, nx): within one cell of a trail
    pls_xy: tuple
    boundary_xy: np.ndarray
    trails_xy: list
    water_xy: list

    @property
    def searchable(self):
        """Cells a drone must actually look at: inside boundary, not water."""
        return self.inside & ~self.water

    def summary(self):
        a = self.cell_m ** 2
        return {
            "grid_shape": tuple(self.inside.shape),
            "cell_m": self.cell_m,
            "area_km2": float(self.inside.sum() * a / 1e6),
            "water_km2": float((self.water & self.inside).sum() * a / 1e6),
            "searchable_km2": float(self.searchable.sum() * a / 1e6),
            "trail_cells": int((self.trail & self.inside).sum()),
        }


def _polyline_mask(points_xy, X, Y, half_width):
    """Mark grid cells whose centre is within half_width of a polyline."""
    mask = np.zeros(X.shape, dtype=bool)
    for (x1, y1), (x2, y2) in zip(points_xy[:-1], points_xy[1:]):
        dx, dy = x2 - x1, y2 - y1
        seg2 = dx * dx + dy * dy
        if seg2 == 0:
            t = np.zeros_like(X)
        else:
            t = np.clip(((X - x1) * dx + (Y - y1) * dy) / seg2, 0, 1)
        d = np.hypot(X - (x1 + t * dx), Y - (y1 + t * dy))
        mask |= d <= half_width
    return mask


def rasterize(area: SearchArea, cell_m: float = 25.0) -> SearchGrid:
    area.validate()
    frame = LocalFrame(*area.pls)

    boundary_xy = np.array([frame.to_local(*p) for p in area.boundary])
    pad = cell_m
    xmin, ymin = boundary_xy.min(axis=0) - pad
    xmax, ymax = boundary_xy.max(axis=0) + pad

    xs = np.arange(xmin + cell_m / 2, xmax, cell_m)
    ys = np.arange(ymin + cell_m / 2, ymax, cell_m)
    X, Y = np.meshgrid(xs, ys)
    pts = np.column_stack([X.ravel(), Y.ravel()])

    inside = Path(boundary_xy).contains_points(pts).reshape(X.shape)

    water_xy = [np.array([frame.to_local(*p) for p in poly]) for poly in area.water]
    water = np.zeros(X.shape, dtype=bool)
    for poly in water_xy:
        water |= Path(poly).contains_points(pts).reshape(X.shape)

    trails_xy = [np.array([frame.to_local(*p) for p in t]) for t in area.trails]
    trail = np.zeros(X.shape, dtype=bool)
    for t in trails_xy:
        trail |= _polyline_mask(t, X, Y, half_width=cell_m / 2)

    return SearchGrid(frame, cell_m, xs, ys, inside, water, trail,
                      pls_xy=(0.0, 0.0), boundary_xy=boundary_xy,
                      trails_xy=trails_xy, water_xy=water_xy)


# ---------------------------------------------------------------------------
# Example scenario (near the PX4 SITL home, ~2.4 km x 1.8 km)
# ---------------------------------------------------------------------------
def example_scenario() -> SearchArea:
    frame = LocalFrame(47.397606, 8.543060)   # SITL home as reference
    L = lambda x, y: frame.to_latlon(x, y)

    boundary = [L(-1200, -900), L(1200, -900), L(1250, 300),
                L(700, 900), L(-700, 900), L(-1200, 400)]
    pls = L(-150, -100)
    trails = [
        [L(-1200, -600), L(-700, -450), L(-150, -100), L(400, 200), L(1000, 350)],
        [L(-150, -100), L(-100, 500), L(100, 880)],
    ]
    lake = [L(500, -700), L(900, -650), L(950, -400), L(650, -300), L(450, -450)]
    return SearchArea(
        name="Example: wooded valley, hiker last seen on trail",
        boundary=boundary, pls=pls,
        subject_category="hiker",
        trails=trails, water=[lake],
    )
