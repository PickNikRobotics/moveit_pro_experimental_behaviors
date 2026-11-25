# Agent comments

Detailed comments by package-relative file and line. Keep entries short.

## include/trajectory_behaviors/check_blend_deviation.hpp

- line 36 (`CheckBlendDeviation`): Method: for each seam between consecutive input trajectories, find the reference
  points within `blend_radius` of the seam, map them to a normalized-time window, and compare each blended point in
  that window with the reference point at the same normalized time. A nearest-neighbor search over the whole path
  would read too low when a blended point passes near some other segment.

## src/check_blend_deviation.cpp

- line 218: Per seam between trajectory k and k+1: (1) locate the connection point in the concatenated reference;
  (2) scan back into k and forward into k+1 for the index range [lo_ref, hi_ref] within blend_radius of it, so the
  zone comes from the reference geometry and blended points count by their normalized time wherever they are; (3)
  convert to [t_lo, t_hi]; (4) compare each blended point in that range with the reference point at the nearest
  index for the same t. Point-to-corresponding-point, not nearest neighbor, so a distant segment cannot hide a
  deviation.

## test/test_behavior_index.cpp

- One JSON field per line allows the comparison to ignore C++ type text. All IDs, metadata and
  other port fields must match. A `null` default means no default; `""` means an empty string.
