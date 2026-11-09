# Phase 5B — Agent-based walker (independent model + fair evaluation)

**Status: Built. Behaviour rules are assumed, not published data.**

## What it adds
- `walker.py` — 20,000 simulated lost hikers start near the PLS, take a correlated random
  walk, are drawn to nearby trails and follow them, can't enter water or leave the boundary,
  and may stop to rest. Final positions give an independent map (POA_B) and a sampler for
  where the hidden person "really" is.
- `walker_params.json` — every parameter, all ASSUMED.
- `run_walker.py` — builds POA_B, compares it with 5A, and runs the independent-truth test.
  Outputs `poa_comparison.png` and `poa_walker.npy`.

## Run
```bash
source ~/sar-drone-env/bin/activate
cd phase5-probability-map && python3 run_walker.py
```

## Result (example scenario)
- 5A vs 5B: correlation 0.71, overlap 0.76.
- Truth is sampled from a walker with DIFFERENT settings and a different seed.
  After covering 20% of the area: uniform order 18.5%, ranked by 5A 31.9%, ranked by 5B 31.6%
  of true locations found.
- The advantage over uniform search is about 13 points, much smaller than the 44% vs 19% in
  the self-consistent 5A plot. That gap is the cost of the model being imperfect.

## Honest limitations
- The truth walker is a variant of the 5B walker, so both maps share the same basic idea
  (trail-following). Real behaviour could differ more, and then the advantage would shrink.
- No elevation, vegetation or slope; the walker moves on a flat grid.
- Walker speed, turning and trail behaviour are assumptions, not calibrated to data.
- Conclusions about real-world benefit are NOT supported yet. This is a simulation test.
