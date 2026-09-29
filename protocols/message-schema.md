# 跨进程通信协议 message-schema

> 版本：v1.0（SLAM 验证阶段）
> 双端实现：`projects/godot/scripts/transport/protocol.gd` ↔ `projects/brain/src/vrobot/comm/messages.py`
> 传输：WebSocket，**Python 端为服务端**（默认 `ws://127.0.0.1:9094`），Godot 为客户端。
> 文本帧 JSON / UTF-8；信封结构所有消息统一。

## 1. 消息信封

```json
{ "type": "sensor_frame", "seq": 1024, "ts": 1727590032000, "payload": { ... } }
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | string | 消息类型，见下表 |
| `seq` | int | 发送端单调递增序号（断线检测/丢帧统计） |
| `ts` | int | 发送时刻 Unix 毫秒 |
| `payload` | object | 各类型自定义内容 |

## 2. 消息类型

| 方向 | type | 频率 | 说明 |
|------|------|------|------|
| Godot → Python | `hello` | 连接后 1 次 | 仿真心跳握手：世界版本、单位、传感器参数 |
| Godot → Python | `sensor_frame` | 20 Hz | **算法唯一输入**：带噪里程计 + 2D 激光 |
| Godot → Python | `ground_truth` | 20 Hz | 真值位姿（仅评估通道，禁止进入算法） |
| Python → Godot | `cmd_vel` | 每 SLAM 帧 / 10 Hz | 速度指令；仅自动模式生效；0.5s 超时未收到则停车 |
| 双向 | `set_mode` | 事件 | Godot → Python：请求切换遥控/自动探索；Python → Godot：连接建立时同步当前模式 |

## 3. 字段定义

### 3.1 hello（上行握手）

```json
{ "type": "hello", "seq": 0, "ts": 0,
  "payload": {
    "world": "sim2d",
    "world_version": 1,
    "unit": "px",
    "sensor": { "lidar_beams": 72, "range_max": 320.0, "range_noise_std": 2.0,
                "odom_noise_std": 0.02 },
    "tick_hz": 60
  } }
```

### 3.2 sensor_frame（上行，算法唯一输入）

```json
{ "type": "sensor_frame", "seq": 1024, "ts": 1727590032000,
  "payload": {
    "odom":  { "x": 320.5, "y": 180.2, "theta": 1.57 },
    "lidar": {
      "ranges": [320.0, 318.5, 12.0, "..."],   // 长度 = lidar_beams，无回波 = -1
      "angle_min": -3.14159,
      "angle_inc": 0.08727,
      "range_max": 320.0
    },
    "collision": false,
    "dt": 0.05
  } }
```

约定：
- **坐标系**：沿用 Godot 像素坐标。x 向右，y 向下；`theta` 即 Godot `rotation`，
  前进方向为 `(cos(theta), sin(theta))`，与 Godot `Vector2.RIGHT.rotated(theta)` 一致
- **odom 噪声**：Godot 每帧在真值速度上叠加高斯噪声后积分得到，与真值解耦；
  `odom_noise_std` 为速度比例噪声系数（对 linear / angular 分别按比例扰动）
- **ranges**：`ranges[i]` 对应射线角 `theta + angle_min + i * angle_inc`（局部角），
  全局角 = `odom.theta + 局部角`；`-1.0` 表示超量程无回波
- **collision**：物理体发生滑动碰撞（供控制层参考，SLAM 不使用）

### 3.3 ground_truth（上行，仅评估）

```json
{ "type": "ground_truth", "seq": 1024, "ts": 1727590032000,
  "payload": { "x": 321.0, "y": 179.8, "theta": 1.5690 } }
```

> 规则：`vrobot.slam.*` 模块 import 该消息类型视为架构违规（评审项）。

### 3.4 cmd_vel（下行）

```json
{ "type": "cmd_vel", "seq": 88, "ts": 1727590032001,
  "payload": { "linear": 60.0, "angular": 0.8 } }
```

| 字段 | 单位 | 范围 |
|------|------|------|
| `linear` | px/s | [-90, 90]（Godot 侧 clamp） |
| `angular` | rad/s | [-3.0, 3.0] |

Godot 侧安全规则：500ms 未收到任何 `cmd_vel` → 速度清零。

### 3.5 set_mode（双向）

```json
{ "type": "set_mode", "seq": 1, "ts": 0,
  "payload": { "mode": "auto" } }   // "auto" | "manual"
```

约定：
- **Godot → Python**：HUD 切换模式时上行请求，Python 端据此动态开关自动探索器
- **Python → Godot**：连接建立时下发一次当前模式，同步 Godot HUD 显示（例如 Python 以 `--auto` 启动时，Godot 侧自动进入 auto）
- 两端收到与当前状态相同的 mode 时忽略，避免乒乓

## 4. 版本与兼容

- 两端各自解析时对未知 `type` 与未知 payload 字段**忽略不报错**（前向兼容）
- 协议破坏性变更：`world_version` / 信封加 `v` 字段，本文件同步更新版本号
