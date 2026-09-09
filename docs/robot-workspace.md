# 家庭机器人工作台

本地入口：<http://127.0.0.1:8766>（服务运行时）。仅监听回环接口。

## 可用功能

- **定位与导航**：现有FAST-LIO、ICP、TF、Smac与MPPI链路说明；细节见 [实现文档](navigation-localization-implementation.md)。
- **导航运行记录**：现有四目标历史结果，明确其简化场景和已知原点条件，不冒充实时运行状态。
- **场景记忆**：点击餐桌、椅子或未收录类别，切换源时间回放。每次查询实际调用已有存储模块，分别展示候选、歧义、过期或未找到。
- **视觉分析**：实际保存的全图、裁剪、上下文与编号反转结果。当前小型VL对编号敏感，不能自动授权运动。

界面为通用机器人产品命名，但当前模式明确为**离线回放、只读、不发送运动指令**。截图、API及浏览器检查见 `experiment/assistant-inspector/parent-verification.json` 和 `robot-workspace.png`。

## 启动

在本集成工作树运行，缓存目录必须不存在：

```bash
python3 experiment/assistant-inspector/server.py \
  --port 8766 --output /tmp/robot-workspace-new-cache
```

不需要ROS、GPU或模型下载。数据读取限定于已跟踪公开图像和已验证语义记忆输入，不提供任意文件读取或导航执行API。

## 周边工具

- [导航恢复快照](nav-baseline-recovery.md)：还原现有导航未提交状态，不修改源worktree。
- [命名地点请求](named-place-request.md)：锁定地图/目录后构造请求，并验证真实ROS消息序列化；不是动作发送器。
- [语义记忆](semantic-memory.md)：独立导入与查询CLI。
- [VL裁剪/上下文小试](research/vlm-crop-context-v1.md)：16次真实调用与失败模式。

这些辅助工具已经合并；跨场景语义坐标映射、实时视觉确认和语音闭环尚未由本页面实现。底层自主导航继续复用原工作区，不另写一套。
