# soarm_localization

`FixedMapRegistration::set_map()` canonicalizes the complete finite structural map once, then publishes one immutable persistent model. The source-cloud limit (`Options::max_cells`, retained for compatibility) is separate from model resources: sparse distance-field payload is capped at 64 MiB and the conservative model allocation estimate at 128 MiB. Failed replacement builds leave no callable partial model.

The model contains a deterministic canonical-cell XY KD index (equal distances select the smaller canonical cell index) and an exact sparse 32x32 tiled XY distance field. Every stored grid value is `min(cap, brute-force full-map squared XY distance)`; missing tiles and cells are exactly `cap`. `model_stats()` reports canonical map cells, tile count, field payload bytes, and successful build count/time.

`search_around(source_local, map_anchor, profile)` scores the deterministic bounded SE(2) lattice with candidate deltas composed on `map_anchor`. It retains lattice endpoints, fixed source denominator, `(score,x,y,yaw)` ordering, and greedy XY NMS. `search()` is the identity-anchor compatibility wrapper.

`refine_bounded_around(source_local, map_anchor, initial_delta, profile)` provides bounded planar refinement around the anchor-composed initial delta. Its inclusive relative x/y/yaw limits are checked before every correspondence and trial evaluation. `refine_bounded()` is the identity-anchor compatibility wrapper. Legacy unbounded GICP remains isolated in `refine()` and is not authoritative for bounded callers.
