# Python 独立大脑方案（二期 · 早期草案）

> **状态：已被《[04-仿真器与算法端架构](04-仿真器与算法端架构.md)》取代**，本文保留作决策沿革记录。
> 与 04 的关键差异：本文规划的是"迁移一期覆盖规划算法到 Python"，实际落地方向调整为"无图 2D SLAM 建图定位先行"（覆盖规划将在 SLAM 地图可用后移植）；进程模型从"Godot 作 WebSocket 服务端"改为"Python 作服务端、Godot 作客户端"（理由见 04 决策 D1）。
> 核心思路（仍有效）：把行为逻辑放到独立 Python 进程，Godot 退化为纯表现层/传感器层，双方经 WebSocket 通信。

## 1. 架构

```
┌─────────────────────┐   WebSocket (JSON/MessagePack, ~20Hz)   ┌──────────────────────┐
│  Godot 4 (表现层)    │ ─────── 传感器: 位姿/碰撞/雷达/电量 ────→ │  brain-python (逻辑)  │
│  场景 / 物理 / 雷达   │ ←────── 指令: MOVE / GOTO / DOCK ────── │  地图/规划/状态机      │
└─────────────────────┘                                          └──────────────────────┘
```

## 2. 规划中的子项目结构（预留，暂不创建）

```
projects/brain-python/
├── pyproject.toml
├── src/brain/
│   ├── main.py              # 入口：启动 WebSocket 服务、主决策循环
│   ├── world_model.py       # 栅格地图（numpy）
│   ├── planner/
│   │   ├── coverage.py      # 弓字形 / 牛耕分解
│   │   └── path_finder.py   # A*（或 networkx）
│   ├── behavior/state_machine.py
│   └── transport/server.py  # aiohttp/websockets 服务端，遵守 protocols/schema
└── tests/
```

## 3. 关键决策点（二期开工前确认）

| 事项 | 候选 | 备注 |
|------|------|------|
| 消息格式 | JSON / MessagePack | 雷达数组大时 MessagePack 更省带宽；schema 定义在 `protocols/` |
| 通信频率 | 20 Hz 决策循环 | 与 Godot 渲染帧率解耦，指令间用最近一次速度保持 |
| Python 依赖 | numpy、websockets/aiohttp、pytest | 尽量少依赖 |
| 时序容错 | 指令超时即 STOP | 网络抖动时机器人安全停车 |
| 同步策略 | Godot 每物理帧打包发送传感器 | 逻辑层 world_model 只依赖传感器流重建地图 |

## 4. 迁移步骤（草案）

1. 一期确保 `core/` 无渲染依赖、接口与 protocols/schema 一致
2. 用 Python 重写 world_model + planner，先在纯单元测试中对拍（同一地图输入，GDScript 与 Python 输出路径一致）
3. Godot 侧启用 `transport/ws_server.gd`，robot_brain 改为透传
4. 联调：先 1Hz 慢速验证状态同步，再提到 20Hz
5. 删除 Godot 内 `core/` 算法实现（保留接口层）
