# projects/brain — Python 算法端（vrobot）

> 职责：只依赖传感器流（带噪里程计 + 2D 激光），自行建图与定位。协议见 `protocols/message-schema.md`，架构见 `docs/04-仿真器与算法端架构.md`。
> 包管理使用 **uv**（`uv sync` 一键就绪，`uv.lock` 锁定版本）；**无图铁律**：`slam/` 模块禁止接触 ground_truth 与任何真实地图。

## 结构

```
src/vrobot/
├── comm/        # WebSocket 服务端（Python=server）+ 消息编解码
├── sensors/     # 传感器数据模型（Odometry2D / LaserScan2D）
├── slam/        # ★ 定位建图核心（纯 numpy，与 IO 解耦）
│   ├── grid_map.py       # 对数概率占据栅格 + Bresenham 射线更新
│   ├── scan_matcher.py   # 相关性扫描匹配（粗→细两级）
│   ├── odom.py           # 里程计增量合成
│   ├── slam2d.py         # 2D SLAM 主类（v1 = TinySLAM 式）
│   └── particle.py       # 预留：RBPF 粒子滤波（v2）
├── control/     # 自主探索：HybridExplorer（沿边→弓形→frontier，默认）/ FrontierExplorer / ReactiveExplorer（兜底）
├── planning/    # A* 栅格寻路（障碍膨胀 / 未知区代价）
├── viz/         # matplotlib 实时地图
├── eval/        # 真值对比评估（位姿误差等）
└── apps/        # 入口 slam_node.py
```

## 自主建图（当前主模式）

行业两阶段：**先探索建图，后弓形覆盖**。本包默认 `hybrid` 模式把两阶段融合为
一个闭环（`HybridExplorer`）：

```
seek_wall 直行找边界 → wall_follow 右手沿边 → 轨迹闭环 → coverage 弓形覆盖
                                                              ⇅（新自由区 ≥120 格时交还）
                                                          frontier 未知边界收尾 → done
```

启动后 Godot 切 Auto 模式即可（终端实时打印阶段切换，"探索完成"即建图完毕）。

```bash
uv run python -m vrobot.apps.slam_node --config configs/slam2d.yaml   # explorer.mode: hybrid
```

可选 `explorer.mode: frontier` 切换为纯 Frontier 探索（步数更少但轨迹无序）。

## 运行

```bash
uv sync                                   # 首次：创建 .venv，按 uv.lock 安装依赖（含项目本身 editable）
uv run python -m vrobot.apps.slam_node --config configs/slam2d.yaml
# 然后启动 Godot 端 projects/godot（主场景 sim2d.tscn），HUD 显示已连接即可遥控
```

## 离线测试（无需 Godot）

```bash
uv run pytest tests/ -v                   # dev 依赖组（pytest）随 uv sync 默认安装
```

## 算法版本

- v1（当前）：里程计预测 + 相关性扫描匹配 + 对数概率占据栅格（TinySLAM 思路）
- v2（预留）：RBPF 粒子滤波（`slam/particle.py`）
- v3（规划）：图优化后端 + 回环检测
