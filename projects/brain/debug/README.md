# 探索建图管线 分阶段隔离调试指南

四个测试脚本分别隔离管线的一段，用受控激励驱动，输出量化 PASS/FAIL。
按 1 → 2 → 3 → 4 顺序执行，哪一段先出 FAIL，问题就在哪一段。

```
Stage 1  Godot 数据通路     （需要 Godot 运行）
Stage 2  cartographer 引擎契约（离线，合成数据）
Stage 3  探索闭环对比        （离线，仿真世界，cartographer vs builtin）
Stage 4  slam_node 全链路插桩（需要 Godot 运行）
```

前置：在 `projects/brain` 目录下用 `.venv/Scripts/python.exe` 运行（脚本可从任意 CWD 执行，已做文件定位引导）。
Stage 2/3/4 默认开启实时地图窗口（地图 + 估计轨迹 + 真值绿+），加 `--no-view` 关闭。
窗口后端自动选择：优先 OpenCV（FastMapViewer，~6ms/帧），未装则回退 matplotlib。
slam_node 主程序同样使用该工厂（`viz/fast_map_viewer.py`）。
Stage 2/3 结束时会暂停等待你查看完最终地图。

---

## Stage 1：Godot 数据通路（debug/stage1_godot.py）

**前置**：Godot 场景运行中，HUD 切到 auto（脚本会下发运动指令）。
**确保 9094 端口空闲**（没有别的 slam_node 在跑）。

```powershell
.venv/Scripts/python.exe debug/stage1_godot.py
```

检查项：
- A 扫描有效率 / 线数与 angle_inc 自洽（单位·约定）
- B 静止 1s 扫描稳定性（噪声水平）
- C 平移约定：指令 (60,0) 2s，真值位移应沿航向
- D 旋转约定：指令 (0,+1.5) 1s，真值 theta 应增大 ~86°
- E 里程计 vs 真值一致性

**判读**：C/D FAIL = 运动学约定镜像/符号错误（最致命）；E FAIL = 里程计标度问题。

---

## Stage 2：cartographer 引擎契约（debug/stage2_carto.py）

**无需 Godot**——合成数据（理想差速驱动 + 解析射线，144 线）直接激励引擎。

```powershell
.venv/Scripts/python.exe debug/stage2_carto.py
```

| 用例 | 激励 | 判据 |
|---|---|---|
| T1 静止 | 60 帧 (0,0) | 位姿漂移 <5px，航向 <2° |
| T2 直线 | 100 帧 (60,0) | 前进 300±30px，横向 <20px |
| T3 原地旋转 | 72 帧 (0,+1.0) | 连续角 +206°，方向为正，幅值误差 <8° |
| T4 正方形闭环 | 4×(200px+90°) | 闭环误差 <60px，free>1500 |

**当前已知结果（2026-10-05 实测）**：
- T1/T2 PASS —— 静止与直行时引擎工作正常
- **T3 FAIL：旋转幅值欠转 ~13.4%**（指令 206.3° 只跟踪到 178.6°，
  误差随转角线性增长；符号正确）。翻转输入 y/θ 验证过，不是镜像约定问题；
  启用在线相关匹配更差（-15.5%）。缺陷在 native 引擎的旋转融合
  （extrapolator / odom 权重），是探索地图角向发散的根源。
- T4 FAIL：90.9px 闭环误差 —— T3 缺陷在四个转角上的累积。

---

## Stage 3：探索闭环对比（debug/stage3_explorer.py）

**无需 Godot**。同一探索任务分别用 cartographer / builtin 引擎闭环，
量化对比建图推进与位姿漂移。

```powershell
.venv/Scripts/python.exe debug/stage3_explorer.py            # 默认 1500 帧（约 10 分钟）
.venv/Scripts/python.exe debug/stage3_explorer.py 800        # 快速版
```

判据：free > 2500、漂移 < 15px/100px、无发散事件。
builtin 是参照基线（此前全量测试：完成、覆盖率 0.98、漂移极小）。
cartographer 若 free 推进正常但漂移大 → 印证 Stage 2 T3 的旋转缺陷
（探索器每转一个弯丢 ~12° → 地图角向发散）。

---

## Stage 4：slam_node 全链路插桩（debug/stage4_live.py）

**前置**：Godot 运行中、HUD auto、9094 端口空闲。

```powershell
.venv/Scripts/python.exe debug/stage4_live.py 3000
```

输出到 `debug_out/`：
- `frames.csv`：逐帧 cmd/估计位姿/真值/地图统计/探索器状态
  （用 Excel 或 pandas 分析：漂移从哪帧开始、与转角/escape 的相关性）
- `map_00300.png` 等：每 300 帧地图快照（直接看"擦亮"效果）

诊断口诀：
- free 增长但 est 与 gt 分叉越来越大 → SLAM 位姿发散（对照 Stage 2 T3）
- `busy=1` 长时间为 1 → 后台规划线程被饿死
- `blacklist` 快速涨到 24 → 观察点被大量误杀
- 反复 `bootstrap=1` → 地图一直空白（桥接断了）

---

## 结论速查（截至 2026-10-05）

| 段 | 状态 | 说明 |
|---|---|---|
| 1 Godot 数据 | 待测 | 脚本就绪 |
| 2 cartographer | **FAIL（已定位）** | 纯旋转角跟踪欠转 ~13.4%·转角，线性增长 |
| 3 探索闭环 | 待测 | 脚本就绪；builtin 基线此前已验证通过 |
| 4 全链路 | 待测 | 脚本就绪；此前实测 free 增长正常、窗口未响应已修复 |

已修复的问题：空图误判完成、卡死拉黑连锁、观察点冷却逻辑、
窗口"未响应"冻结（事件泵）、概率插入器参数（0.55/0.49 → 0.75/0.30）。
