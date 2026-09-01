# Fixed-map registration core

This package is a no-runtime localization-core checkpoint.  It does not create ROS topics, TF, Nav2, or FAST-LIO integration.

`FixedMapRegistration::set_map()` voxelizes a finite bounded fixed map and builds a fixed XY distance field. `search()` performs serial deterministic bounded SE(2) correlative scoring **before** GICP: every finite source cell contributes to a fixed denominator, out-of-field cells receive a capped loss, candidates are ordered by `(score, x, y, yaw)`, and greedy XY spatial NMS retains distinct peaks. It returns raw peak scores, best/second score gap, cell/evaluation counts, and elapsed search time.

Ambiguity is deliberately caller policy: this package neither embeds nor tunes a score-gap cutoff. `refine_selected()` accepts only caller-selected pre-search peaks and invokes small_gicp afterwards; refinement outputs cannot modify the pre-search peaks, ordering, or gap.

The independent SE(2) observability spectrum is raw diagnostic data. Callers supply the relative rank tolerance through `Observability::rank_for_relative_tolerance()`.
