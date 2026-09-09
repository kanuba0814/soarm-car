# Native Livox Converter

`simulation.launch.py` keeps the Python `SimAdapter` point path by default. Set `use_native_livox_converter:=true` to start `soarm_native_livox_converter`; in that mode the adapter does not subscribe to the source cloud or publish `/mid360/points` and `/livox/lidar`.

The native converter accepts only little-endian, contiguous PointCloud2 layouts with finite Float32 `x`, `y`, and `z` fields. It republishes the PointCloud2 unchanged except for `header.frame_id=livox_frame` and emits the generated Livox `CustomMsg`. Invalid layouts are dropped before either output is published.

`throttle_odom_path:=true` independently limits the RViz-only `/odom_path` output to 2 Hz when it has subscribers. It does not alter `/odom` or TF, and pose history remains bounded at 2000 entries.

Validation uses the captured `raw-cloud.cdr` artifact from the Intel renderer diagnostic. The converter test deserializes it using ROS generated serialization, checks PointCloud2 payload preservation, and compares the serialized Livox output against an independent reproduction of the existing Python conversion behavior.
