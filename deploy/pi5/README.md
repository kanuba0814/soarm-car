# Raspberry Pi 5 robot deployment

These assets target a Raspberry Pi 5 running **Raspberry Pi OS 64-bit Desktop**. They build an arm64 Ubuntu 22.04 / ROS 2 Humble image containing the WheelTEC base (`serial`, `wheeltec_robot_msg`, and `turn_on_wheeltec_robot`), legacy Orbbec Astra (`astra_camera_msgs`, `astra_camera`), Livox MID-360 (`livox_ros_driver2`), and FAST-LIO Localization (`fast_lio_localization`) packages. The existing Intel-specific `docker/Dockerfile` is not used or modified.

The image is deliberately idle by default: it opens a Bash shell and launches no hardware drivers.

## First boot

1. Connect the HDMI touchscreen and its USB touch lead. Raspberry Pi OS handles this HDMI display plus USB HID combination; no display overlay is required.
2. Clone this repository to the Pi, then run the bootstrap as root:
   ```bash
   sudo ./deploy/pi5/bootstrap-host.sh
   ```
   It installs Docker Engine and Compose from Docker's apt repository without prompts, installs the WheelTEC controller and Orbbec USB udev rules, configures `eth0` as `10.42.0.2/24` with gateway and DNS at `10.42.0.1`, and builds the image. Wi-Fi is deliberately not configured.
3. Sign out and back in before using Docker without `sudo`. Connect the Ethernet peer at `10.42.0.1/24`; the Fedora host shares Internet access.

## Build and idle shell

The bootstrap builds the image. Rebuild it after source changes:

```bash
docker compose -f deploy/pi5/compose.yaml build
```

Start an idle shell (the default command never starts a driver):

```bash
docker compose -f deploy/pi5/compose.yaml run --rm wheeltec-base
```

Set a ROS domain at launch time if required:

```bash
ROS_DOMAIN_ID=42 docker compose -f deploy/pi5/compose.yaml run --rm wheeltec-base
```

The Compose service uses host networking and passes `/dev/wheeltec_controller` plus `/dev/bus/usb`; it does **not** use privileged mode.

## Hardware launches

Only run one of these after hardware, robot model, map, and safety conditions have been checked. None are started by Compose or the image entrypoint.

### WheelTEC base

**Put the wheels off the ground before launching the driver.** The vendor driver can command the chassis when it receives ROS traffic.

With the controller plugged in, verify the host alias and its properties:

```bash
ls -l /dev/wheeltec_controller
udevadm info --query=property --name=/dev/wheeltec_controller
```

Then launch it only after confirming the robot model and safety conditions:

```bash
docker compose -f deploy/pi5/compose.yaml run --rm wheeltec-base \
  ros2 launch turn_on_wheeltec_robot base_serial.launch.py
```

The vendor launch file uses `/dev/wheeltec_controller` at `115200` baud. The image health check validates that the installed `turn_on_wheeltec_robot` package is discoverable and intentionally does not open the serial device.

### Orbbec Astra camera

Connect a supported legacy Astra camera, confirm its host USB permissions, then start the vendor launch explicitly:

```bash
docker compose -f deploy/pi5/compose.yaml run --rm wheeltec-base \
  ros2 launch astra_camera astra_pro.launch.py
```

### Livox MID-360 and FAST-LIO Localization

The MID-360 needs a real, separate `10.*` lidar subnet and lidar address selected **after hardware boot**. Do not use the deployment link subnet for the lidar. The vendor `MID360_config.json` is left unchanged: set the real lidar network/address only after the hardware is connected and its assigned values are known. No runtime network validation is performed by this deployment.

After that configuration is complete, launch the driver in one terminal:

```bash
docker compose -f deploy/pi5/compose.yaml run --rm wheeltec-base \
  ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

In a second terminal, start FAST-LIO with the installed `mid360.yaml` and RViz disabled:

```bash
docker compose -f deploy/pi5/compose.yaml run --rm wheeltec-base \
  ros2 launch fast_lio_localization mapping.launch.py \
  config_file:=mid360.yaml rviz:=false
```

A real map and calibrated lidar-to-robot extrinsics remain required before localization can produce meaningful results.
