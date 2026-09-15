# Dabai DC1：算法对照与融合建议

日期：2026-09-13。本文为主源研究与实验建议，不是新增实测结果。已知基线来自当前任务：两段办公室 RGB-D bag 的 ORB-SLAM3 有效跟踪率为 65.6% / 87.8%，失跟后存在多图合并和坐标跳变；bag 没有 IMU、轮速或轨迹真值。

## 优先实测什么

| 顺序 | 对照方案 | 要回答的问题 | 限制 |
|---|---|---|---|
| 1 | OpenCV `RgbdOdometry`、`ICPOdometry`、`RgbdICPOdometry` | 光度、深度几何及二者联合，哪种能覆盖 ORB 失跟片段？ | 帧间里程计，不等同于带重定位和闭环的完整 SLAM |
| 2 | Open3D RGB-D Hybrid，多尺度 + 帧间步长 1/2/3 | 多尺度和时间采样是否提高快速转动、模糊片段的可用率？ | 成功返回与低残差不证明全局准确 |
| 3 | RTAB-Map RGB-D 默认视觉前端，然后视觉 + ICP 消融 | 换完整 SLAM 系统后，局部跟踪、恢复及闭环表现是否改善？ | 依赖较重；不能把 ICP 当作必然改进 |

OpenCV 官方提供光度与 ICP 能量联合最小化，并开放金字塔层数、梯度阈值、深度差阈值和随机像素子集比例。因此它适合低成本做三路消融，不应把三个类包装成三个完整 SLAM 系统。[OpenCV 类参考](https://docs.opencv.org/4.13.0/d2/d0f/classcv_1_1rgbd_1_1RgbdICPOdometry.html)

Open3D 官方实现帧间 RGB-D 运动估计；Hybrid 使用光度与深度约束，tensor API 提供多尺度计算。它与 OpenCV 的比较主要是不同实现与优化策略的对照，不能宣称为完全独立的传感器证据。[RGB-D 教程](https://open3d.org/docs/latest/tutorial/pipelines/rgbd_odometry.html)、[多尺度 API](https://open3d.org/html/cpp_api/odometry_2_r_g_b_d_odometry_8h.html)

RTAB-Map 包含建图、定位、回环检测与图优化。其作者明确说明 RGB-D 的 ICP 在部分场景会改善精度，也可能引入难以察觉的错误；低几何环境尤其需要小心。对本办公室，白墙会同时削弱纹理与某些几何自由度的约束，所以“深度补视觉”只是待验证假设。[系统流程](https://introlab.github.io/rtabmap/api/latest/classrtabmap_1_1Rtabmap.html)、[作者 ICP 分析](https://github.com/introlab/rtabmap/wiki/ICP)

## “多种采样融合”可行，但分三种

1. **同一帧多尺度、不同像素采样**：用粗层扩大优化收敛范围，细层精化；可以比较均匀、梯度和深度边缘采样。它改变计算分配，不产生新观测；重复纹理、错误标定和深度偏差仍共享。推荐先使用已有多尺度实现，避免先造复杂融合器。[OpenCV](https://docs.opencv.org/4.13.0/d2/d0f/classcv_1_1rgbd_1_1RgbdICPOdometry.html)、[Open3D](https://open3d.org/html/cpp_api/odometry_2_r_g_b_d_odometry_8h.html)
2. **不同时间跨度、多算法候选**：短跨度保重叠，长跨度可提供更明显运动，但也会减小重叠。建议作为多假设与故障检测：统一到相同起止时刻，比较正反向一致性、重投影/几何残差、有效深度和退化情况；通过门控后选择或联合优化。不要直接平均全局轨迹，尤其不能跨 ORB 地图重置平均。此处是工程建议，不是官方保证。
3. **真正多传感器**：后续补录 IMU 和轮速，才有不同测量机制帮助度过视觉失效。`robot_localization` 支持多源状态估计，但官方提醒避免重复信息。现有 bag 无这些数据，因此本轮只能测试 RGB-D 内部互补，不能声称完成视觉惯性或轮视觉融合。[robot_localization](https://docs.ros.org/en/rolling/p/robot_localization/)、[重复信息说明](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst)

**统计限制（推断）**：ORB、直接法和 ICP 均源于同一 DC1 RGB-D，其误差存在共同成分；同一序列不同频率/像素子集同样相关。把各自位姿当独立观测灌入 EKF 或独立因子，会有重复计数、协方差过于乐观的风险。优先选择有质量门控的主备切换；若做紧耦合，明确原始残差、噪声与相关性，而不是把“算法数量”当“独立证据数量”。因子图负责表达测量约束与噪声模型，不自动解决错误或重复观测。[GTSAM 原理](https://gtsam.org/tutorials/intro.html)

## 建议架构和验证约束

局部连续运动估计与全局地图校正分层：RGB-D 局部前端（未来加入 IMU/轮速）负责连续里程计，SLAM 回环负责地图坐标修正。不能把地图合并跳变解释为瞬时真实运动，也不应靠平滑掩盖失跟。[robot_localization 状态估计配置](https://docs.ros.org/en/kinetic/api/robot_localization/html/state_estimation_nodes.html)

本轮统一输入与评分：保持配对、深度单位、裁剪和内参一致；记录算法返回率、最长有效连续段、失败时刻、相对位姿突变、计算 p50/p95，以及失效帧 RGB/深度叠加。各算法的 success 定义不同，应另设同一套运动与残差门控并同时报告原始返回率。算法间一致、正反向一致和残差只能作诊断代理，不能替代真值 ATE/RPE；地图几何看着整齐也不等于轨迹准确。

推荐最小实验矩阵：先跑 OpenCV 三路 × 两个 bag 的完整帧率，再仅对联合方法做步长 2/3 消融。若已有 Open3D 环境，追加 Hybrid 作为实现对照。RTAB-Map 单列完整 SLAM 对照，不与帧间成功率直接排名。先看到互补失效区间，再决定是否值得实施融合。
