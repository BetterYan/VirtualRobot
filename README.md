# VirtualRobot — 扫地机器人仿真

Godot 可视化 + 可外置的机器人行为逻辑（覆盖路径规划）。

- 📄 全部规划与设计文档：[docs/](docs/00-项目总览.md)
- 🎮 Godot 子项目：[projects/godot/](projects/godot/README.md)（一期，当前阶段）
- 🐍 Python 独立大脑：[projects/brain-python/](projects/brain-python/README.md)（二期预留）
- 🔌 通信协议契约：[protocols/](protocols/README.md)

## 快速了解

1. 先读 [docs/00-项目总览.md](docs/00-项目总览.md) —— 架构与演进路线
2. 再读 [docs/01-godot起步方案.md](docs/01-godot起步方案.md) —— 一期里程碑、目录、接口、验收标准
3. `Robot.blend` 为机器人模型源文件，Godot 4 可直接导入（需本机安装 Blender）

## 当前进度

- [x] 顶层结构与文档规划
- [x] M1~M6 Godot 项目代码就绪（`projects/godot`，待编辑器首次运行验证）
- [ ] 首次运行验证 + 调参（扫道间距、速度、电量速率）
- [ ] 二期：Python 大脑 + WebSocket
