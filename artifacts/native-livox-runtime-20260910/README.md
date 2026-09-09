# Native Livox Runtime Comparison

Environment: image `sha256:1da9a18609c4664d7441d9f334c95e971f0795aa3815f6707ed761f2ce93d227`, Intel `/dev/dri/card1` and `/dev/dri/renderD128`, Mesa 25.1.7, headless OGRE. No NVIDIA/CUDA variables or devices were used. Both runs used an immutable recovery from `41463f9` plus only the converter/path overlay, the archived room/map/TF inputs, `throttle_odom_path:=false`, and no action goals.

The synthetic benchmark replays the actual captured `raw-cloud.cdr` at 10 Hz wall clock. It preserves the captured source stamp (`201.4 s`), publishes all 11,520 source points, and checks every received PointCloud2 and CustomMsg against the Python reference after generated ROS deserialization. Native and Python each completed 100 messages. Native median Livox receipt latency was 0.0337 s versus 0.0898 s for Python; node CPU was 0.28 CPU-seconds versus 5.65 CPU-seconds over the ten-second replay.

The full runtime observer saw 101 raw and 101 adapted clouds for native mode, both `720 x 16`; median source-clock lag was 0.019 s adapted versus 0.018 s raw. Baseline saw 101 raw but 63 adapted clouds, with adapted median lag 1.016 s versus raw 0.014 s. Native mode observed 505 Livox IMU receipts in 25 seconds versus 63 in baseline.

Both full runs were blocked before navigation by the same existing `map -> base_footprint` transform absence. The renderer evidence and source pipeline are valid, but this is not a navigation success result. No safety, Nav2, gate, map, footprint, TF, LiDAR FOV/range/rate, or timestamp code was modified.
