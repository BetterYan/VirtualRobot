# 跨进程通信协议（共享契约）

> 状态：占位。M3 前完成初版 schema。
> 本目录是与语言无关的唯一通信契约，`projects/godot` 与 `projects/brain-python` 共同遵守。

计划内容：
- 传感器上行消息：`sensor`（位姿、碰撞、雷达命中点、电量）
- 指令下行消息：`move` / `goto` / `dock` / `stop`
- 消息信封：`{ "type": ..., "seq": ..., "ts": ..., "payload": ... }`
- 序列化格式与频率：见 `docs/02-python独立大脑方案.md` 第 3 节
