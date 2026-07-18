# Investigation: Why A* Pathfinding Is Slow / Endless on the Iran Map

**Date:** 2026-07-18
**Component:** `src/missile/planning/cpp/pathfinder.cpp`, driven by `src/missile/planning/pathfinding_backend.py`
**Symptom:** Pathfinding on the Iran DEM (39,601 × 39,601 px) is dramatically slower than Siberia or the SRTM tile — and often never returns — *even when the start→end distance is smaller or comparable*.

---

## TL;DR

Three compounding causes, in order of impact:

1. **The heuristic is far too weak relative to the edge cost.** Every move pays a `height_penalty = neighbor_elev × 0.8`, but the heuristic only estimates straight-line *distance*. On the high Iranian plateau the height term is **~24× larger** than the distance term, so `f ≈ g` and A* collapses into uniform-cost (Dijkstra) search — it floods outward by area instead of aiming at the goal. Distance between endpoints is almost irrelevant; **explored area** is what explodes.
2. **Iran is a high plateau, Siberia/SRTM are low.** Because the penalty is proportional to *absolute* elevation, the g-vs-h imbalance is worst exactly where the terrain is highest. Same algorithm, much worse behaviour.
3. **The map is huge and the A* state arrays don't fit in RAM.** Iran needs ~12.5 GB just for `g_score` + `came_from`, plus a 6.3 GB float32 DEM, on a 16 GB machine → swap thrashing, which is what turns "slow" into "endless."

---

## Evidence

Measured from the actual DEMs in `data/dem/` (elevation from a decimated sample; resolution/geometry from the GeoTIFF transform):

| Map | Size | Pixels | Mean elev | Dist cost/step | `height_penalty`/step | **hp / dist ratio** | A* arrays (`g`+`came_from`) | DEM as f32 |
|-----|------|--------|-----------|----------------|-----------------------|---------------------|------------------------------|------------|
| **Iran** (filled) | 39,601 × 39,601 | **1,568 M** | ~926 m* | ~31 m | ~741 | **24.0×** | **12.5 GB** | 6.3 GB |
| Siberia | 39,601 × 21,601 | 855 M | ~408 m | ~31 m | ~326 | 10.6× | 6.8 GB | 3.4 GB |
| SRTM 43_02 | 6,000 × 6,000 | 36 M | ~164 m | ~93 m | ~131 | 1.4× | 0.3 GB | 0.1 GB |

*Machine RAM: ~17.2 GB total.*
*\*926 m is a decimated-sample mean; the Iranian interior plateau/ridges the missile actually crosses sit at 1,500–3,000 m+, so the real in-search ratio is worse than 24×.*

The single most telling column is **hp / dist ratio**: it tracks the observed pain almost perfectly. SRTM (1.4×) is fast, Siberia (10.6×) is tolerable, Iran (24×) is pathological.

---

## Root Cause #1 — The heuristic doesn't model the dominant cost (A* degenerates to Dijkstra)

`get_movement_cost()` (pathfinder.cpp:197) charges, per step:

```
dist_cost                          // ~31 m on Iran
+ height_penalty = neighbor_elev*0.8   // ~741–2400 on the plateau  ← dominant
+ slope_penalty  = |Δh| * 5            // large on rough terrain
+ gradient penalty (up to climb*100)   // large uphill
```

But `heuristic()` (pathfinder.cpp:183) returns **only the straight-line distance in meters**:

```cpp
return sqrt(dist_x*dist_x + dist_y*dist_y);   // ~tens of meters per step
```

A* orders the frontier by `f = g + h·w`. The heuristic accounts for the ~31-unit distance term and is completely blind to the ~741-unit height term (and the slope/gradient terms). So on Iran:

```
g per step ≈ 31 + 741 + …  ≈ 800+
h per step ≈ 31
→  h / (true step cost) ≈ 4%
→  f ≈ g
```

When `h ≪ g`, A* has no sense of direction and behaves like **uniform-cost search**: it expands a roughly circular flood outward from the start until it happens to engulf the goal. The number of nodes expanded scales with the **area** of that flood, not with the endpoint distance. **This is why a "smaller distance" doesn't help** — a short straight-line distance still forces the frontier to fill a huge high-cost basin before the goal's `f` bubbles to the top of the queue.

By contrast, on SRTM the height term (~131) is comparable to the distance term (~93), so `h` is a meaningful fraction of the true cost, A* stays directional, and the search terminates quickly.

Two secondary aggravators inside the same loop:

- **No stale-node skip.** The queue is a lazy-deletion `priority_queue` with duplicate pushes and no decrease-key. On pop there's no `if (current.f_score > g_score[curr_idx]) continue;` guard (pathfinder.cpp:77), so already-finalized nodes get re-expanded and their 8 neighbours re-scanned. Under a Dijkstra-like flood of 10⁸–10⁹ pops this is a large constant-factor multiplier on both time and heap memory.
- **`heuristic_weight` can't rescue it.** The default is 2.0 (`find_path` in pathfinding_backend.py:79), i.e. weighted A*. But weighting a heuristic that is ~4% of the true cost still leaves it ~8% — negligible. The knob has "currently no difference lol" noted in the docstring for exactly this reason.

## Root Cause #2 — Absolute elevation makes the plateau the worst case

`height_penalty = neighbor_elev × 0.8` is proportional to *absolute* altitude, so:

- On the **Iranian plateau** every single step costs 800–2,400 in penalty regardless of whether the terrain is locally flat. There is no cheap "downhill drain" direction anywhere near the start, so the frontier spreads **isotropically** and the open set balloons. The cost field is essentially a large positive constant plus small ripples → the search is dominated by *number of steps*, i.e. pure breadth.
- On **Siberian lowlands / SRTM**, penalties are 2–6× smaller and there are genuine low-elevation corridors, so even the weak heuristic plus the natural cost gradient keeps the frontier from filling the whole map.

So the same code that is merely slow on Siberia becomes intractable on Iran purely because Iran is *high*, and the metric punishes altitude in absolute terms.

## Root Cause #3 — The map doesn't fit in RAM (slow → "endless")

`Pathfinding.__init__` (pathfinding_backend.py:35) converts the int16 DEM to **float32** for C++, and `find_path` allocates two dense `total_pixels` arrays (pathfinder.cpp:62–65):

- `g_score` (float32) + `came_from` (int32) = **8 bytes × 1.568 B px = 12.5 GB**
- float32 DEM copy = **6.3 GB** (the original int16 read is another 3.1 GB held by Python during construction)

That is ~19 GB of live working set on a **17 GB** machine before the search even starts expanding nodes — and the lazy priority queue then grows on top of it. The result is continuous swapping: forward progress continues but at disk speed, which presents to the user as **hung / endless**. This matches the existing note that *full-tile Iran A\* OOMs 16 GB*. Siberia (6.8 GB + 3.4 GB ≈ 10 GB) stays under the limit; SRTM is trivial.

Note these three causes multiply: because #1/#2 force the flood to expand orders of magnitude more nodes, the search *touches* far more of those 12.5 GB of state (defeating the OS's ability to keep the hot set resident), which makes #3's swapping far worse than a well-directed search over the same map would ever trigger.

---

## Recommended fixes (highest leverage first)

1. **Make the heuristic consistent with the cost metric, or the metric consistent with the heuristic.** The clean fix is to stop adding an *absolute-elevation* penalty per step. Options:
   - Replace `height_penalty = elev*0.8` with a term the heuristic can also estimate (e.g. fold "stay low" into a one-time preprocessed cost field, or drop it and rely on the slope/gradient penalties which are already relative to Δh).
   - If "prefer low ground" must stay, add a matching admissible lower-bound to `heuristic()` (e.g. `distance + 0.8 × max(0, elev_min_along_line)`), so `h` grows with `g`. Without this, no amount of tuning stops the Dijkstra collapse.
2. **Add a stale-node skip on pop** (`if (current.f_score > g_score[curr_idx] + eps) continue;`) — a few lines, removes redundant re-expansions.
3. **Tile / window the search** rather than allocating full-map arrays: run A* on a cropped bounding box around start↔end (with margin), or downsample the DEM for a coarse pass then refine. This directly fixes the 12.5 GB allocation and is consistent with the existing "full-tile A* OOMs" note.
4. **Keep the DEM in int16** in C++ (cast per-access) to halve the DEM footprint, or memory-map it.
5. **Use a real weighted-A*/JPS or a bucketed queue** once #1 is fixed and the heuristic is actually informative.

Fix #1 is the one that explains the Iran-specific behaviour and unblocks the other maps' scaling as well; #3 is what turns "hours + swap" into "seconds" on the full tile.
