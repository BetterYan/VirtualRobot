# VirtualRobot 🤖

**扫地机器人覆盖路径规划仿真** — 基于 Godot 4 的可视化仿真，行为逻辑与渲染层完全解耦，可独立演进。

[![Godot](https://img.shields.io/badge/Godot-4.7-478CBF?logo=godotengine&logoColor=white)](https://godotengine.org)
[![Stage](https://img.shields.io/badge/阶段-一期%20Godot%20一体式-orange)](docs/00-项目总览.md)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

---

## ✨ 功能特性

- **弓字形覆盖规划（CPP）**：蛇形清扫序列生成 + A\* 断点衔接 + 循迹执行
- **全局避障**：基于已知静态地图的全局规划，物理碰撞兜底
- **电量管理与回充**：低电 20% 触发 A\* 回桩，充满续扫，全覆盖后归位
- **双版本可视化**：2D 版（Line2D 轨迹、覆盖热力图、HUD）与 3D 版（3D 房间、小地图、仪表盘）
- **逻辑与渲染零耦合**：`core/` 算法层不依赖任何渲染节点，为"大脑外置"预留迁移路径

## 🏗️ 架构

```
┌─────────────────────────────────────────────┐
│  一期（当前）：Godot 一体式                    │
│  ┌──────────────┐      ┌──────────────────┐ │
│  │ core/ 算法层  │ ───▶ │ visualization/   │ │
│  │ 覆盖规划·A*   │ 指令 │ 2D/3D 渲染·HUD   │ │
│  │ 状态机·电量   │ 数据 │ 热力图·小地图     │ │
│  └──────────────┘      └──────────────────┘ │
└─────────────────────────────────────────────┘
                  │ 二期演进
                  ▼
┌────────────────┐   WebSocket    ┌───────────┐
│ brain-python   │ ◀────────────▶ │ Godot     │
│ Python 独立大脑 │  protocols/    │ 仅表现层   │
└────────────────┘   消息契约     └───────────┘
```

关键原则：一期写逻辑时就保证 `core/` 不依赖任何渲染节点，二期迁移只换宿主、不改接口。

## 🚀 快速开始

### 环境要求

- [Godot 4.7+](https://godotengine.org/download)（标准版即可，无需 .NET）
- 可选：Blender（仅在需要编辑 `asserts/Robot.blend` 模型时安装）

### 运行

1. 用 Godot 4.7 打开 `projects/godot/project.godot`（首次打开会自动导入资源、生成 `.godot` 缓存）
2. 按 `F5` 运行 → 点击左下角「开始」按钮
3. 机器人开始弓字形清扫，HUD 实时显示状态、覆盖率、电量

### 无头单元测试

```bash
godot --headless --path projects/godot --script res://tests/run_tests.gd
```

## 📁 项目结构

```
VirtualRobot/
├── asserts/
│   └── Robot.blend            # 机器人模型源文件（Godot 4 可直接导入）
├── docs/                      # 设计与规划文档
│   ├── 00-项目总览.md          # 全局架构与演进路线
│   ├── 01-godot起步方案.md     # 一期规划：里程碑、接口、验收标准
│   └── 02-python独立大脑方案.md # 二期架构与依赖选型
├── projects/
│   ├── godot/                 # Godot 4 主项目（一期）
│   │   ├── scenes/            # main.tscn（2D）/ main_3d.tscn（3D）
│   │   ├── scripts/
│   │   │   ├── core/          # 算法层：覆盖规划、A*、状态机、世界模型
│   │   │   ├── visualization/ # 2D 渲染：轨迹、热力图、HUD
│   │   │   └── visualization3d/ # 3D 渲染：房间、小地图、仪表盘
│   │   └── tests/             # 无头单测
│   └── brain-python/          # 预留：Python 独立行为逻辑进程（二期）
└── protocols/                 # 跨进程通信协议契约（语言无关）
```

## 🗺️ 路线图

- [x] 顶层结构与文档规划
- [x] M1 场景搭建：房间、墙体/家具碰撞、充电桩、机器人实体
- [x] M2 传感器与快照：位姿/碰撞/雷达字段，HUD 实时显示
- [x] M3 弓字形覆盖：蛇形序列 + A\* 断点衔接 + 循迹执行
- [x] M4 避障与绕行：全局规划 + 物理碰撞兜底
- [x] M5 电量与回充：低电回桩、充满续扫、全覆盖归位
- [x] M6 可视化：轨迹、覆盖热力图、HUD、开始/暂停/重置
- [ ] 首次运行验证 + 调参（扫道间距、速度、电量速率）
- [ ] 二期：Python 独立大脑 + WebSocket（协议见 `protocols/`）

## 📚 文档

| 文档 | 内容 |
|------|------|
| [docs/00-项目总览.md](docs/00-项目总览.md) | 架构、目录约定、演进路线 |
| [docs/01-godot起步方案.md](docs/01-godot起步方案.md) | 一期目标、里程碑、接口设计、验收标准 |
| [docs/02-python独立大脑方案.md](docs/02-python独立大脑方案.md) | 二期架构、进程模型、依赖选型 |
| [protocols/README.md](protocols/README.md) | WebSocket 消息 schema 契约 |

## License

本项目基于 [MIT License](LICENSE) 开源。
