# Official ROS Humble Jammy ros-base publishes a linux/arm64/v8 variant.
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Build and runtime dependencies declared by the WheelTEC, Orbbec, Livox, and
# FAST-LIO package manifests. libuvc-dev is available for Jammy arm64.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libapr1-dev \
    libboost-system-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    libopencv-dev \
    libpcl-dev \
    libpython3-dev \
    libuvc-dev \
    python3-colcon-common-extensions \
    ros-humble-ackermann-msgs \
    ros-humble-ament-cmake-auto \
    ros-humble-camera-info-manager \
    ros-humble-cv-bridge \
    ros-humble-image-geometry \
    ros-humble-image-publisher \
    ros-humble-image-transport \
    ros-humble-magic-enum \
    ros-humble-nav2-msgs \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rclcpp-components \
    ros-humble-rcl-interfaces \
    ros-humble-rcutils \
    ros-humble-rosidl-default-generators \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-turtlesim \
    ros-humble-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# livox_ros_driver2 requires Livox-SDK2 in /usr/local. Pin the SDK rather than
# inheriting a moving branch or tag.
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
    && cd /tmp/Livox-SDK2 \
    && git checkout --detach f5d9375f84efe2b15bc0a052d3e18482ed13adf4 \
    && test "$(git rev-parse HEAD)" = "f5d9375f84efe2b15bc0a052d3e18482ed13adf4" \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --parallel 2 \
    && cmake --install build \
    && rm -rf /tmp/Livox-SDK2

# The Astra vendor CMake uses these historical /usr/local paths. Keep the
# source unmodified apart from architecture detection and bridge them to the
# Jammy arm64 libuvc package locations.
RUN ln -s /usr/include/libuvc /usr/local/include/libuvc \
    && ln -s /usr/lib/aarch64-linux-gnu/libuvc.so /usr/local/lib/libuvc.so \
    && test -e /usr/local/include/libuvc/libuvc.h \
    && test -e /usr/local/lib/libuvc.so

WORKDIR /opt/wheeltec_ws
# Vendor directories are supplied by Compose as additional BuildKit contexts,
# avoiding the root build context's intentional vendor exclusions.
COPY --from=wheeltec_vendor serial_ros2 src/serial_ros2
COPY --from=wheeltec_vendor wheeltec_robot_msg src/wheeltec_robot_msg
COPY --from=wheeltec_vendor turn_on_wheeltec_robot src/turn_on_wheeltec_robot
COPY --from=wheeltec_vendor ros2_astra_camera/astra_camera_msgs src/astra_camera_msgs
COPY --from=wheeltec_vendor ros2_astra_camera/astra_camera src/astra_camera
COPY --from=localization_vendor livox_ros_driver2 src/livox_ros_driver2
COPY --from=localization_vendor FAST_LIO_LOCALIZATION_ROS2 src/FAST_LIO_LOCALIZATION_ROS2

RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --parallel-workers 2 \
        --packages-select \
            serial \
            wheeltec_robot_msg \
            turn_on_wheeltec_robot \
            astra_camera_msgs \
            astra_camera \
            livox_ros_driver2 \
            fast_lio_localization

COPY deploy/pi5/entrypoint.sh /entrypoint.sh
COPY deploy/pi5/healthcheck.sh /healthcheck.sh
RUN chmod 0755 /entrypoint.sh /healthcheck.sh

ENTRYPOINT ["/entrypoint.sh"]
# Deliberately do not launch drivers: attaching hardware must not move wheels.
CMD ["bash"]
