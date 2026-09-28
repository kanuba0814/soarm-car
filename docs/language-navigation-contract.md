# 自然语言导航与抓取接口标准

## 当前闭环

`text` 或已由本地 ASR 产生的 `voice_transcript` → `POST /api/intent/preview` → 严格地点白名单 → `map`/position-only 请求。

解析结果必须先是 `awaiting_confirmation`，且 `motion_authorized: false`。只有上层动作所有者在确认地图、定位健康、单动作锁和 Nav2 action server 可用后，才能调用既有 `NavigateToPose` 传输；解析器本身不得创建 action client、发布速度或绕过 `prepare.py --verify-prepared`。

当前支持的目标是锁定的 `room-1`、`room-2`、`room-3`、`mian`。探索、巡航、朝向、多地点、否定句和未知地点拒绝。`voice_transcript` 只表示转写边界，不宣称本模块完成音频 ASR。

## 抓取接口（预留，不执行）

```json
{
  "task": "pick",
  "object_id": "opaque-perception-id",
  "source_observation_id": "obs-id",
  "grasp_frame": "map",
  "pose": {"position": {"x": 0, "y": 0, "z": 0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
  "confidence": 0.0,
  "requires_human_confirmation": true
}
```

抓取消费者必须验证对象仍可见、位姿时间戳未过期、碰撞/夹爪/急停健康，并以 `proposed → confirmed → executing → succeeded|failed|aborted` 状态机返回证据。导航到物体不等于抓取成功；未有机械臂控制器、碰撞验证和真实传感器证据前，接口只能返回 `not_implemented`。
