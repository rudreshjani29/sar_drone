# Phase 5A — Probability-of-Area (POA) Map (baseline)

**Status: Baseline built. Parameters are ILLUSTRATIVE, not real ISRID data.**

## Goal
Turn the Phase 4 grid into a probability map: how likely is the subject to be in each cell?
Phase 6+ uses it to weight drone allocation.

## Layers
1. **Distance prior** — 2-D Rayleigh decay around the Point Last Seen, fitted to one
   percentile anchor (`lpb_params.json`).
2. **Trail attraction** — multiplier based on distance to trails (Phase 4 geometry).
3. **Bayesian update** — `bayes_update()` lowers probability in searched cells by the
   probability of detection (POD) and renormalizes. Used by later phases for live re-planning.

## Files
- `poa_model.py` — prior, Bayesian update, capture curve
- `lpb_params.json` — all model parameters with provenance notes
- `run_poa.py` — builds the map, runs sanity checks, writes `poa_map.png` and `poa_prior.npy`

## Run
```bash
source ~/sar-drone-env/bin/activate
cd phase5-probability-map && python3 run_poa.py
```

## Honest limitations
- **The hiker-specific ISRID/Koester table was not available**, so the distance model uses one
  published anchor (ISRID "temperate flat", 75% within 2 km, as quoted by Sava et al. 2015),
  not the hiker numbers. Another source (Mitchell 1984, hikers, 66% within 2 miles) disagrees
  with it, which shows how uncertain this is. Replace with the real table when you have it.
- Trail weight (3.0) and corridor width (100 m) are **assumed**. Only the direction is
  supported (Mitchell 1984: 75% of hikers followed a trail or drainage at some point).
- The example area (~4 km²) is much smaller than the real LPB search zones, so the distance
  decay is mild inside it.
- The right-hand plot assumes the model is correct. It is NOT evidence the method beats
  uniform search in reality. Phase 5B evaluates with an independent agent-based walker.
- No elevation, vegetation, or slope.

## Next: Phase 5B
Agent-based walker simulation as an alternative/independent prior, then compare against
this baseline.
