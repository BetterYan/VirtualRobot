extends RefCounted
## 静态配置：房间、栅格、机器人、电量参数（纯数据，无场景依赖）

# --- 栅格 ---
const CELL := 20              # 格子边长 px
const COLS := 45
const ROWS := 32
const ROOM_SIZE := Vector2(COLS * CELL, ROWS * CELL)  # 900 x 640

# --- 机器人 ---
const ROBOT_RADIUS := 14.0    # px（直径 28，可穿过 20px 栅格走廊的整数倍通道）
const MOVE_SPEED := 90.0      # px/s
const TURN_SPEED := 6.0       # rad/s
const ARRIVE_DIST := 6.0      # 判定到达路点的距离 px

# --- 电量 ---
const BATTERY_DRAIN := 0.004     # 清扫时每秒消耗（满电约 250s）
const BATTERY_DOCK_DRAIN := 0.002
const BATTERY_CHARGE := 0.05     # 充电时每秒恢复
const BATTERY_LOW := 0.2         # 低于此值触发回充

# --- 家具障碍（世界坐标 Rect2）---
const FURNITURE := [
	Rect2(180, 140, 140, 100),
	Rect2(520, 120, 160, 90),
	Rect2(300, 380, 100, 160),
	Rect2(640, 420, 140, 120),
]

# --- 充电桩 ---
const CHARGER_CELL := Vector2i(2, 29)
const START_CELL := Vector2i(4, 4)
