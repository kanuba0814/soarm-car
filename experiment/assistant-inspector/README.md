# 本地毕业助手检查器

该实验是只读、离线的实现检查器，不启动 ROS、GPU 或导航，也不发送运动指令。

```bash
python3 experiment/assistant-inspector/server.py \
  --port 8765 \
  --output /tmp/soarm-assistant-inspector-cache
```

仅监听 `127.0.0.1`。`--output` 必须是不存在的新目录；服务只会在其中创建由自身拥有的语义记忆缓存。启动时通过现有 `semantic_memory.core.import_public_archive` 校验并导入公开归档，查询通过同一模块的 `query` 完成。HTTP 接口和公开 PNG 均为代码中的白名单，未提供目录浏览、文件路径参数或任何写入接口。

检查页面可用的 API：`/api/overview`、`/api/query?category=餐桌&replay=fresh` 和 `/api/vlm`。其他路径、附加参数、非白名单类别或回放模式均返回可见的 JSON 错误。
