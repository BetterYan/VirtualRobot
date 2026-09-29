# protocols/ — 跨进程通信契约

> 与语言无关的唯一消息定义：**[message-schema.md](message-schema.md) v1.0**
> 两端实现：`projects/godot/scripts/transport/protocol.gd` ↔ `projects/brain/src/vrobot/comm/messages.py`
> 传输：WebSocket（Python 服务端 ws://127.0.0.1:9094，Godot 客户端），JSON 文本帧。

消息类型：`hello` / `sensor_frame` / `ground_truth`（上行），`cmd_vel` / `set_mode`（下行）。
改动流程：先改 message-schema.md 并升版本号 → 两端同步实现。
