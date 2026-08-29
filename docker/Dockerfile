ARG GRAPHICS_IMAGE=local/arc140t-focal-dri:mesa-25.1.7
FROM ${GRAPHICS_IMAGE} AS host_graphics

FROM osrf/ros:humble-desktop-jammy
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    libdrm2 \
    libegl1 \
    libelf1 \
    libexpat1 \
    libgbm1 \
    libgl1 \
    libglvnd0 \
    libunwind8 \
    libx11-xcb1 \
    libxcb-dri2-0 \
    libxcb-dri3-0 \
    libxcb-glx0 \
    libxcb-present0 \
    libxcb-randr0 \
    libxcb-sync1 \
    libxcb-xfixes0 \
    libxshmfence1 \
    mesa-utils \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-yaml \
    ros-humble-foxglove-bridge \
    ros-humble-joint-state-publisher-gui \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-robot-state-publisher \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# Reuse the already verified local Mesa 25.1.7 runtime for the host's Intel 0x7d51 GPU.
COPY --from=host_graphics /opt/mesa /opt/mesa
COPY --from=host_graphics /opt/spirv-llvm /opt/spirv-llvm
COPY --from=host_graphics /opt/spirv-tools /opt/spirv-tools
COPY --from=host_graphics /usr/lib/x86_64-linux-gnu/libLLVM-18* /usr/lib/x86_64-linux-gnu/
COPY --from=host_graphics /usr/lib/x86_64-linux-gnu/libLLVM.so.18.1 /usr/lib/x86_64-linux-gnu/
COPY --from=host_graphics /usr/lib/x86_64-linux-gnu/libffi.so.7* /usr/lib/x86_64-linux-gnu/
ENV ARC140T_MESA_VERSION=25.1.7 \
    LD_LIBRARY_PATH=/opt/mesa/lib/x86_64-linux-gnu:/opt/spirv-llvm/lib/x86_64-linux-gnu:/opt/spirv-tools/lib/x86_64-linux-gnu:/usr/lib/llvm-18/lib \
    LIBGL_DRIVERS_PATH=/opt/mesa/lib/x86_64-linux-gnu/dri \
    __EGL_VENDOR_LIBRARY_FILENAMES=/opt/mesa/share/glvnd/egl_vendor.d/50_arc140t_mesa.json \
    __GLX_VENDOR_LIBRARY_NAME=mesa
RUN mkdir -p /opt/mesa/share/glvnd/egl_vendor.d \
    && printf '%s\n' '{' '  "file_format_version": "1.0.0",' '  "ICD": { "library_path": "/opt/mesa/lib/x86_64-linux-gnu/libEGL_mesa.so.0" }' '}' \
       > /opt/mesa/share/glvnd/egl_vendor.d/50_arc140t_mesa.json \
    && printf '%s\n' \
       /opt/mesa/lib/x86_64-linux-gnu \
       /opt/spirv-llvm/lib/x86_64-linux-gnu \
       /opt/spirv-tools/lib/x86_64-linux-gnu \
       > /etc/ld.so.conf.d/arc140t-mesa.conf \
    && ldconfig

WORKDIR /workspaces/soarm-car
COPY ros2_ws/src /workspaces/soarm-car/ros2_ws/src
RUN source /opt/ros/humble/setup.bash && cd ros2_ws && colcon build --symlink-install
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
