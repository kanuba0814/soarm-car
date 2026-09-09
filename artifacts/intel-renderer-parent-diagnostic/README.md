# 实际Gazebo Intel渲染与CPU链路采样

此为父进程直接采样，不是glxinfo替代证据。通过已提交恢复工具生成 `/tmp/soarm-intel-renderer-inspect-20260910`，用固定镜像 `sha256:1da9a18609c4664d7441d9f334c95e971f0795aa3815f6707ed761f2ce93d227` 启动原 `house_fastlio_nav2.launch.py`。没有发送initialpose、导航目标或速度，因此不是导航成功测试。诊断容器 `soarm-intel-renderer-inspect` 已停止。

实际启动环境：DRI `/dev/dri`（当前仅Intel card1/renderD128），group 39/105，ROS_DOMAIN_ID=95，IGN_PARTITION=parent_intel_renderer_20260910，OMP/OPENBLAS/MKL/NUMEXPR线程均1，未设置DISPLAY、NVIDIA/CUDA或软件渲染变量；复用原镜像环境和导航参数文件。

## 已观测

- `ogre2-headless.log` 直接复制自运行容器 `/root/.ignition/rendering/ogre2.log`：EGL设备 `/dev/dri/card1`，GL_VENDOR=Intel，GL_RENDERER=Mesa Intel(R) Graphics (ARL)，Mesa25.1.7。
- `driver-maps-fds.txt`：在Gazebo进程存活时读取其 `/proc/3196/maps` 和fd。
- `cpu-sample.json`：在容器内读取各 `/proc/<pid>/stat` 的utime+stime，相隔约2秒，按SC_CLK_TCK计算单核百分比。窗口内Gazebo约182%，sim_adapter约99%，odom_path约99%；不是长期均值。
- `topic-rates.json`：一个独立轻量rclpy观察器，10秒墙钟内计数clock/odom/points，记录源时间跨度，观察RTF约0.358。订阅使用sensor_data QoS，因此消息计数不等于无损发布计数。
- `cloud-lag.json`：同一观察器5秒墙钟内记录原始桥接点云和适配点云的源stamp、最近clock与monotonic接收时间。原始点云15条，中位clock-stamp约0.018秒，最大0.030秒；适配点云10条，中位约1.0095秒，最大1.072秒。单位是仿真秒，不是墙钟端到端延迟。
- `raw-cloud.cdr`：用实际 `rclpy.serialization.serialize_message` 保存的一条原始桥接PointCloud2，供等价转换测试，不是生成的合成点云。

实际输入布局：width720、height16、point_step32、row_step23040、data368640字节、frame `kuruma/livox_frame/mid360`；x/y/z是FLOAT32，offset0/4/8；intensity offset16，ring offset24。

## 结论边界

实际OGRE已证实硬件Intel，不再仅凭GLX推断。数据支持检查Python逐点转换、同执行器IMU阻塞和高频完整轨迹序列化这两个CPU热点；后续优化必须保留点内容/顺序/时间戳及传感器FOV/范围/更新率，并用这条真实CDR和运行中时序复验。这里没有证明优化已完成，也不改动任何安全超时或导航参数。
