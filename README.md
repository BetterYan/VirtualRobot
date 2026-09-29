# VirtualRobot 🤖

**机器人仿真与定位算法验证平台** — Godot 4 无图传感器仿真 + Python 端 2D SLAM；一期覆盖清扫演示保留可运行。

[![Godot](https://img.shields.io/badge/Godot-4.7-478CBF?logo=godotengine&logoColor=white)](https://godotengine.org)
[![Stage](https://img.shields.io/badge/阶段-SLAM%20验证(无图模式)-blue)](docs/04-仿真器与算法端架构.md)
[![Tests](https://img.shields.io/badge/tests-15%20passed-success)](projects/brain)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

---

## ✨ 功能特性

**SLAM 验证模式（当前主模式）**
- **无图传感器仿真**：Godot 构建物理世界（墙体/家具/碰撞），输出带噪 2D 激光（144 线）与带噪里程计，不向算法端泄漏任何地图
- **2D SLAM（Python 实现）**：里程计预测 + 相关性扫描匹配（双线性插值、粗→细两级）+ 对数概率占据栅格 + 关键帧插入（防旋转涂抹）
- **混合自主探索建图（默认）**：沿边探测（直行找边界 → 右手沿边 → 轨迹闭环）→ 弓形覆盖已知自由区 → Frontier 收尾未知边界，三阶段交替直至地图完整（合成世界验证：召回 100%、覆盖率 99.9%）
- **纯 Frontier 探索（可选）**：A* 导航驶向"已知/未知边界"，目标承诺制 + 停滞保护（约 100s 完成全屋，墙面召回 99.4%）
- **双驾驶模式**：算法端自动探索 / Godot 键盘遥控，cmd_vel 0.5s 超时安全停车
- **实时可视化**：matplotlib 占据概率地图 + 估计轨迹 + 真值对照；真值走独立评估通道，代码级隔离
- **离线可验证**：合成理想世界端到端测试（合成巡航：终点误差 2.4px/0.4°；自主建图：召回 99.4%、误报 1.5%）

**一期覆盖清扫（保留演示）**
- 弓字形覆盖规划（CPP）+ A* 断点衔接、电量管理与回充、2D/3D 双版本可视化

## 🏗️ 架构

```
┌──────────────────────────┐  WebSocket(JSON) 20Hz ┌───────────────────────────┐
│ Godot 4 = 虚拟物理世界     │ ── sensor_frame ────▶ │ Python vrobot = 算法端      │
│ · 场景/物理碰撞            │  带噪里程计+2D激光      │ · 2D SLAM：建图 + 定位      │
│ · 传感器仿真（雷达/里程计） │ ◀── cmd_vel ─────────  │ · 自动探索 / 遥控执行       │
│ · 键盘遥控 / 模式切换      │                       │ · 实时地图可视化            │
│ · 轨迹渲染（操作员视角）    │ ── ground_truth ────▶ │ · 真值评估（不进算法）       │
└──────────────────────────┘   仅评估，可选           └───────────────────────────┘
```

- **无图铁律**：算法端永远不知道世界的真面目，只从带噪传感器流重建认知
- **契约先行**：消息格式唯一真相源在 `protocols/message-schema.md`，两端各自实现
- **一期一体式**（`core/` 内嵌算法 + 2D/3D 渲染）保留可运行，作"开天眼"对照基线

## 🚀 快速开始

### 环境要求

- [Godot 4.7+](https://godotengine.org/download)（标准版即可，无需 .NET）
- [uv](https://docs.astral.sh/uv/)（Python 包管理，Python ≥ 3.10 由 uv 自动准备；依赖：numpy / websockets / pyyaml / matplotlib）
- 可选：Blender（仅在需要编辑 `asserts/Robot.blend` 模型时安装）

### 运行（SLAM 验证模式，当前主模式）

```bash
# 1. 启动 Python 算法端（先启动，监听 ws://127.0.0.1:9094）
cd projects/brain
uv sync                                   # 首次：创建 .venv 并按 uv.lock 安装依赖
uv run python -m vrobot.apps.slam_node --config configs/slam2d.yaml

# 2. Godot 端：打开 projects/godot → F5（主场景 sim2d.tscn，无图传感器仿真）
#    HUD 显示"已连接"后：方向键/WASD 遥控采集数据，或点按钮切 Auto 让算法端驾驶
```

### 运行（一期一体式覆盖清扫演示，保留）

1. Godot 打开 `projects/godot`，运行 `scenes/main.tscn` 或 `main_3d.tscn`
2. 点击「开始」→ 弓字形清扫 + 回充全流程

### 测试

```bash
# Python 算法端（无需 Godot，合成世界端到端验证）
cd projects/brain && uv run pytest tests/ -v

# Godot 一期无头单测
godot --headless --path projects/godot --script res://tests/run_tests.gd
```

## 📁 项目结构

```
VirtualRobot/
├── asserts/
│   └── Robot.blend            # 机器人模型源文件（Godot 4 可直接导入）
├── docs/                      # 设计与规划文档
│   ├── 00-项目总览.md          # 全局架构与演进路线
│   ├── 03-机器人定位算法调研.html # 定位算法技术调研（2026）
│   └── 04-仿真器与算法端架构.md # ★ 当前架构：无图仿真 + SLAM 验证
├── projects/
│   ├── godot/                 # Godot 4 虚拟世界（无图传感器仿真器）
│   │   ├── scenes/            # sim2d.tscn（主）+ main.tscn / main_3d.tscn（保留）
│   │   └── scripts/
│   │       ├── sim/           # ★ 传感器仿真：雷达/里程计/机器人/遥控/HUD
│   │       ├── transport/     # WebSocket 客户端 + 协议编解码
│   │       ├── core/          # 一期算法（保留作对照）
│   │       └── visualization*/ # 一期 2D/3D 渲染（保留）
│   └── brain/                 # Python 算法端（vrobot 包）
│       ├── src/vrobot/
│       │   ├── slam/          # ★ 2D SLAM：栅格建图 + 扫描匹配
│       │   ├── comm/          # WebSocket 服务端 + 消息编解码
│       │   ├── control/       # 自动探索（雷达避障）
│       │   ├── viz/           # matplotlib 实时地图
│       │   └── apps/          # 入口 slam_node.py
│       ├── configs/           # 算法参数
│       └── tests/             # 合成世界端到端测试
└── protocols/                 # 跨进程通信协议契约（语言无关）
    └── message-schema.md      # ★ v1.0 消息定义
```

## 🗺️ 路线图

- [x] 一期：Godot 一体式覆盖清扫（M1–M6，保留可运行）
- [x] 定位算法调研与选型（docs/03）
- [x] 无图仿真架构 + 通信协议 v1.0（docs/04 + protocols/）
- [x] Python 2D SLAM v1：扫描匹配 + 占据栅格（合成巡航：终点误差 2.4px，召回 100%）
- [x] Godot sim2d 传感器仿真场景 + Godot ↔ Python 实机联调
- [x] Frontier 自主探索建图：无人遥控自动获得完整地图（召回 99.4%，误报 1.5%）
- [ ] 覆盖规划移植回 Python 端（建图完成后自动切入弓形清扫，对应行业两阶段流程）
- [ ] v2：RBPF 粒子滤波 → v3：图优化回环
- [ ] 3D / VSLAM 扩展（sensors/slam3d）

## 📚 文档

| 文档 | 内容 |
|------|------|
| [docs/00-项目总览.md](docs/00-项目总览.md) | 架构、目录约定、演进路线 |
| [docs/03-机器人定位算法调研.html](docs/03-机器人定位算法调研.html) | 2026 定位算法全景调研与选型 |
| [docs/04-仿真器与算法端架构.md](docs/04-仿真器与算法端架构.md) | ★ 当前架构、算法管线、扩展规划 |
| [protocols/message-schema.md](protocols/message-schema.md) | WebSocket 消息 schema 契约 v1.0 |

## License

本项目基于 [MIT License](LICENSE) 开源。
