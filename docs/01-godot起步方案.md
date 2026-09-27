# Godot 起步方案（一期实现规划）

> 阶段目标：在 Godot 4 内完成"房间 + 扫地机器人 + 自动清扫路径 + 轨迹可视化"的一体式可运行版本。行为逻辑与表现层严格分离，为二期外置 Python 大脑留好接缝。

## 1. 目标与范围

**做**：
- 2D 俯视角房间场景（墙体、家具障碍、充电桩），栅格化
- 机器人实体（`Robot.blend` 导入或先用圆形占位），物理碰撞
- 传感器仿真：位姿、碰撞信号、简化激光雷达（射线扫描）
- 覆盖路径规划：沿墙定界 → 牛耕分区 → 弓字形 → A* 跨区转移
- 状态机：待机 / 清扫 / 避障 / 回充
- 可视化：实时轨迹线、已清扫格子热力图、覆盖率/电量 HUD

**不做**（留到二期或迭代）：
- Python 外置大脑、WebSocket 通信（接口已预留，见 protocols/）
- 3D 视角切换、真实雷达点云、灰尘粒子特效

## 2. 里程碑

| 里程碑 | 内容 | 验收标准 |
|--------|------|---------|
| M1 场景搭建 | 房间 TileMap、墙体/家具静态体、机器人实体与碰撞、Blend 导入 | 场景可运行，机器人可被手动拖动或按键移动 |
| M2 传感器与手动驾驶 | RayCast2D 雷达、碰撞信号、位姿读取；按键直接控制机器人 | 能在 HUD 上看到实时位姿与雷达命中数 |
| M3 弓字形覆盖 | world_model 栅格化 + coverage_planner 生成扫道 + 循迹执行 | 在无障碍房间完成完整弓字形，覆盖率 100% |
| M4 避障与分区 | 障碍绕行、牛耕分解、A* 跨区转移 | 带家具房间覆盖率 ≥ 95%，无卡死 |
| M5 电量与回充 | 电量模型、低电量中断清扫、A* 回桩、充完续扫 | 全流程自动运行至覆盖率达标 |
| M6 可视化完善 | 轨迹 Line2D、覆盖热力图、HUD 统计、启停/重置按钮 | 演示可一键开始/重置，统计准确 |

## 3. Godot 子项目目录结构

```
projects/godot/
├── project.godot
├── icon.svg
├── scenes/
│   ├── main.tscn                # 主场景：房间 + 机器人 + HUD
│   ├── room/room.tscn           # 房间：TileMap + 静态碰撞体 + 充电桩
│   ├── robot/robot.tscn         # 机器人：模型/占位 + CharacterBody2D + 传感器区
│   └── ui/hud.tscn              # 覆盖率、电量、状态、按钮
├── scripts/
│   ├── visualization/           # ── 表现层：只渲染，不含算法
│   │   ├── main.gd              # 组装、主循环调度、演示控制
│   │   ├── robot_view.gd        # 消费速度指令 → 驱动实体移动
│   │   ├── trail_renderer.gd    # Line2D 轨迹 + 覆盖格子着色
│   │   ├── hud.gd               # 统计显示
│   │   └── sensors.gd           # 挂在机器人上：产出传感器数据（纯数据）
│   ├── core/                    # ── 行为逻辑层：纯逻辑，禁止 import 渲染类
│   │   ├── robot_brain.gd       # 顶层调度：传感器进 → 指令出（二期被 Python 替换）
│   │   ├── world_model.gd       # 栅格地图：障碍层 + 覆盖标记层 + 覆盖率统计
│   │   ├── config.gd            # 地图尺寸、机器人半径、扫道间距、电量参数
│   │   ├── planner/
│   │   │   ├── coverage_planner.gd   # 弓字形扫道生成
│   │   │   ├── cell_decomposer.gd    # 牛耕区域分解
│   │   │   └── path_finder.gd        # A* 栅格寻路
│   │   └── behavior/
│   │       ├── behavior_state.gd     # 状态机：IDLE/CLEAN/AVOID/DOCK
│   │       ├── wall_follower.gd      # 沿墙跟随定界
│   │       └── battery_manager.gd    # 电量模型与回充触发
│   └── transport/               # ── 预留：二期通信层（一期空置）
│       ├── ws_server.gd         # WebSocket 服务端（二期启用）
│       └── protocol.gd          # 与 protocols/message-schema.md 对应的序列化
├── assets/                      # 房间贴图、UI 素材（Robot.blend 在仓库根目录）
└── tests/
    ├── test_world_model.gd
    ├── test_coverage_planner.gd # 给定地图断言路径覆盖所有可行格
    └── test_path_finder.gd
```

## 4. 核心接口设计（一期定死，二期沿用）

### 4.1 传感器上行（Godot 世界 → robot_brain）

```
SensorSnapshot:
  pose: Vector2(x, y) + heading: float     # 位姿（世界坐标 + 朝向角）
  collision: bool                           # 本帧是否碰撞
  lidar_hits: Array[Vector2]               # 简化雷达命中点（机器人局部坐标）
  battery: float                            # 0.0 ~ 1.0
```

### 4.2 指令下行（robot_brain → Godot 执行器）

```
Command:
  type: MOVE        # 线速度 linear: float, 角速度 angular: float
  type: GOTO        # 目标栅格 cell: Vector2i（由表现层换算为循迹）
  type: DOCK        # 回充电桩
  type: STOP
```

### 4.3 数据归属

- **world_model（栅格 + 覆盖标记）归逻辑层所有**——它是机器人对世界的"认知"，不是 Godot 场景的镜像
- 表现层只读两样东西做渲染：轨迹位姿历史、覆盖标记矩阵
- 这样二期换成 Python 大脑时，只是把 4.1/4.2 的数据搬上 WebSocket，渲染端零改动

## 5. 技术要点

- **Blend 导入**：Godot 4 编辑器内直接拖入 `Robot.blend`（需本机装有 Blender）；一期也可先用 `robot.tscn` 里的圆形占位，模型后补
- **栅格化**：房间以 TileMap 绘制，同时用 `config.gd` 定义的格宽（建议 16~32px）建二维数组；机器人半径决定扫道间距（约 0.9 × 直径）
- **循迹执行**：规划器输出的是栅格路径点序列；`robot_view.gd` 用"朝向目标点 + 速度指令"的方式走点，不用瞬移，保证轨迹自然
- **雷达仿真**：从机器人中心向周围打 N 条（如 36 条）`RayCast2D`/`intersect_ray`，命中点直接喂给逻辑层，供沿墙与避障使用
- **测试**：planner 与 path_finder 是纯函数式算法类，用 GUT 或 Godot 4.4 内置测试跑单测，不依赖场景

## 6. 验收标准（一期整体）

1. 一键启动：机器人自动完成 沿墙 → 分区 → 弓字 → 回充 全流程
2. 带障碍房间覆盖率 ≥ 95%，无卡死、无穿墙
3. HUD 实时显示覆盖率、电量、当前状态；轨迹线与热力图正确
4. `core/` 目录下无任何对 `Node2D`/场景节点的引用（可用 grep 验证）
5. 算法单测通过

## 7. 下一步

- 按 M1 开始搭建 `projects/godot` 骨架（project.godot、场景、占位脚本）
- M3 前补齐 `protocols/message-schema.md` 的初版消息定义（一期内部使用同样结构，提前对齐契约）
