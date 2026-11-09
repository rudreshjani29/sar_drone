# Phase 4 — Search Area Layout

**Status: Done (hand-defined scenario)**

## Goal
Define *where* the search happens and turn it into numpy grids that Phase 5
(probability-of-area map) and Phase 6+ (multi-drone allocation) can consume.

## What's Here
- `search_area.py` — `SearchArea` (boundary polygon, Point Last Seen, subject
  category label, trails, water), local metre projection around the PLS, and
  `rasterize()` which produces boolean grids: `inside`, `water`, `trail`, and
  `searchable` (inside and not water). Scenarios save/load as JSON.
- `plot_search_area.py` — renders `search_area_layout.png` and writes
  `example_scenario.json`.
- Example: ~4.1 km² wooded valley, 3.9 km² searchable, 25 m cells, hiker PLS on a trail junction.

## Run
```bash
source ~/sar-drone-env/bin/activate
pip install numpy matplotlib
cd phase4-search-area-layout
python3 plot_search_area.py
```

## Limitations
- The area is hand-defined. A VTOL/survey-derived area is not implemented.
- `subject_category` is only a label; Lost Person Behavior statistics come in Phase 5.
- Terrain is trails and water only (no elevation, vegetation, or slope).
- Equirectangular projection: fine for a few km, not for large areas.

## Next
Phase 5 builds the probability-of-area map on top of this grid.
