# small_gicp provenance

This package vendors [small_gicp](https://github.com/koide3/small_gicp) under its upstream MIT license (`upstream/LICENSE`).
The source is exactly release `v1.0.1`, annotated-tag peeled commit `57c1106daf83c2c79ee0c58a9c7ed0032298ff4e`.

The upstream CMake Eigen `FetchContent` fallback is replaced with `find_package(Eigen3 CONFIG REQUIRED)` so Docker and colcon builds have no network build path. TBB, CUDA, examples, benchmarks, tests, and Python bindings are disabled; only Eigen and OpenMP are used.
