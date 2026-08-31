# soarm_astra

A single ROS-facing contract for the physical Orbbec Astra S and the Gazebo Fortress RGB-D approximation.

## Contract

Both backends publish:

- `/camera/color/image_raw` (`sensor_msgs/Image`, RGB);
- `/camera/color/camera_info`;
- `/camera/depth/image_raw` (`16UC1`, millimetres);
- `/camera/depth/camera_info`;
- `/camera/depth/points` (`sensor_msgs/PointCloud2`).

The image and point-cloud frame IDs use `camera_color_optical_frame` and `camera_depth_optical_frame`. The physical driver may additionally expose device services, IR, extrinsics, and registered colour point clouds; algorithms intended for automatic hardware/simulation testing must not require those extra interfaces unless their tests explicitly skip the simulation backend.

## Physical camera

The verified development device is `2bc5:0402`, product string `ASTRA S`, USB 2.0 high speed. From the repository root:

```bash
sudo ./scripts/install-astra-udev.sh
./scripts/dev-astra-start.sh
./scripts/dev-astra-smoke.sh
```

The container builds the official legacy OpenNI ROS 2 driver from the commit pinned in `docker/Dockerfile`. The host udev rule is required for non-root deployments. Hardware scripts resolve the current `2bc5:0402` bus/device path from sysfs, and `compose.astra.yaml` passes only that character device into Docker.

## Gazebo

The standard simulation launch automatically starts the compatibility adapter:

```bash
./scripts/dev-astra-sim-smoke.sh
```

Gazebo's `rgbd_camera` is an algorithm-test approximation, not a calibrated Astra S optical/noise model. It does not simulate IR, USB disconnects, exposure controls, device services, or measured distortion. Record calibration and failure behaviour from real hardware before claiming hardware validation.
