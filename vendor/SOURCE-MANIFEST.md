# Pinned upstream sources

- Orbbec official ROS 2 wrapper: https://github.com/orbbec/OrbbecSDK_ROS2.git
  - branch: `v2-main`
  - commit: `8e7cad2bfa2c4a6ac4e779be99c64e72166043af`
  - SDK binary: `orbbec_camera/SDK/lib/x64/libOrbbecSDK.so.2.9.3`
  - SDK binary SHA-256: `9cf22925e025db010427513f5eb7f1f1db994f0b992855688f4f123f37815537`
  - The official udev rules in this revision include Dabai DC1 `2bc5:0557` and `2bc5:0657`.

- ORB-SLAM3 upstream: https://github.com/UZ-SLAMLab/ORB_SLAM3.git
  - branch: `master`
  - commit: `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`
  - vocabulary archive SHA-256: `ff2d0e82a69a8f4c5c002e3a0dff82a00e5872e1659fac1b996f41166b92693b`
  - Official upstream documents ROS 1 integration, not a ROS 2 package. The ROS 2 adapter remains isolated and must not publish operational `map -> odom`.

- Pangolin upstream: https://github.com/stevenlovegrove/Pangolin.git
  - branch: `master`
  - commit: `dd801d244db3a8e27b7fe8020cd751404aa818fd`
  - Built without Python bindings; used only as the pinned ORB-SLAM3 viewer/build dependency.

## Verified DC1 recording runtime (2026-09-13)

The SDK v2 build above did not enumerate this DC1. Recording now uses the pre-existing local
`soarm-car-astra-s-ros2-dev:latest` image, `/opt/astra_ros2`, package `astra_camera` (OpenNI),
with `dabai_dc1.launch.xml uvc_product_id:=0x0557 depth_registration:=true enable_ir:=false
 depth_height:=400 tf_publish_rate:=0.0 enable_point_cloud:=false`.
The exact image ID is stored in `config/dabai-dc1-profile.json` and each recording's
`artifacts/perception-dev/dabai-dc1/<name>/image-id.txt`. The source commit of this pre-existing
image is not established here; do not confuse it with the SDK v2 revision above.
