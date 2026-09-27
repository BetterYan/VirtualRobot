# projects/godot — Godot 4.7 主项目（一期）

> 可视化层 + 一期内嵌行为逻辑。规划详见 `docs/01-godot起步方案.md`。

当前状态：**M1~M6 代码全部就绪，待在编辑器中首次运行验证**。

## 运行方式

1. 用 Godot 4.7.2 打开 `projects/godot/project.godot`（首次会自动导入资源、生成 .godot 缓存）
2. F5 运行 → 点击左下角「开始」按钮
3. 无头单测（命令行）：
   ```
   godot --headless --path projects/godot --script res://tests/run_tests.gd
   ```

## 已实现（对照里程碑）

- [x] M1 场景搭建：900×640 房间、墙体/家具静态碰撞、充电桩、机器人圆形占位实体
- [x] M2 传感器与快照：位姿/碰撞/预留雷达字段，HUD 实时显示
- [x] M3 弓字形覆盖：coverage_planner 蛇形序列 + A* 断点衔接 + 循迹执行
- [x] M4 避障与绕行：已知静态地图全局规划，物理碰撞兜底
- [x] M5 电量与回充：低电 20% 触发 A* 回桩、充满续扫、全覆盖后归位 DONE
- [x] M6 可视化：Line2D 轨迹、覆盖热力图、状态/覆盖率/电量 HUD、开始/暂停/重置

## 架构约束（二期迁移前提）

- `scripts/core/**` 禁止引用任何渲染节点类——迁移 Python 大脑时整体搬走
- `scripts/visualization/**` 只消费指令与数据，不含算法
- `scripts/transport/`（二期启用）尚未创建，接口见 `docs/01-godot起步方案.md` 第 4 节
