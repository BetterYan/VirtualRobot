extends Node2D
## 无图传感器仿真主场景（sim2d）。
##
## 职责：构建物理世界（墙体/家具）→ 机器人执行运动 → 雷达/里程计产出带噪
## 传感器流 → WebSocket 上行给 Python 算法端；下行 cmd_vel 在 auto 模式驱动机器人。
## 本场景不向算法端提供任何地图或真值位姿（ground_truth 仅评估通道）。

const Protocol = preload("res://scripts/transport/protocol.gd")
const Teleop = preload("res://scripts/sim/teleop.gd")
const SimRobot = preload("res://scripts/sim/sim_robot.gd")
const Lidar2D = preload("res://scripts/sim/lidar2d.gd")
const OdomModel = preload("res://scripts/sim/odom_model.gd")
const SimHud = preload("res://scripts/sim/sim_hud.gd")
const WSClient = preload("res://scripts/transport/ws_client.gd")

const WS_URL := "ws://127.0.0.1:9094"
const SEND_HZ := 20.0
const BEAMS := 144
const RANGE_MAX := 320.0
const RANGE_NOISE := 2.0
const ODOM_NOISE := 0.02

# 房间与家具布局（与 core/config.gd 一致，仅供仿真器自己知道）
const ROOM_SIZE := Vector2(900, 640)
const WALL_T := 14.0
const FURNITURE := [
	Rect2(180, 140, 140, 100),
	Rect2(520, 120, 160, 90),
	Rect2(300, 380, 100, 160),
	Rect2(640, 420, 140, 120),
]
const START_POS := Vector2(100, 90)

var robot: SimRobot
var lidar: Lidar2D
var odom: OdomModel
var hud: SimHud
var ws: WSClient

var mode := "manual"   # "manual" | "auto"
var _seq := 0
var _mode_seq := 0
var _send_acc := 0.0
var _send_hz_meas := 0.0
var _frames_this_sec := 0
var _hz_timer := 0.0
var _truth_trail := PackedVector2Array()


func _ready() -> void:
	_build_physics()
	_build_robot()
	_build_comms()
	hud = SimHud.new()
	add_child(hud)
	hud.mode_toggle_requested.connect(_toggle_mode)


# ---------- 场景构建 ----------

func _build_physics() -> void:
	var w := ROOM_SIZE.x
	var h := ROOM_SIZE.y
	_add_static_rect(Rect2(-WALL_T, -WALL_T, w + WALL_T * 2, WALL_T))
	_add_static_rect(Rect2(-WALL_T, h, w + WALL_T * 2, WALL_T))
	_add_static_rect(Rect2(-WALL_T, 0, WALL_T, h))
	_add_static_rect(Rect2(w, 0, WALL_T, h))
	for rect in FURNITURE:
		_add_static_rect(rect)


func _add_static_rect(rect: Rect2) -> void:
	var body := StaticBody2D.new()
	var shape := CollisionShape2D.new()
	var r := RectangleShape2D.new()
	r.size = rect.size
	shape.shape = r
	body.position = rect.get_center()
	body.add_child(shape)
	add_child(body)


func _build_robot() -> void:
	robot = SimRobot.new()
	robot.position = START_POS
	add_child(robot)

	lidar = Lidar2D.new()
	lidar.beams = BEAMS
	lidar.range_max = RANGE_MAX
	lidar.noise_std = RANGE_NOISE
	lidar.angle_step = TAU / BEAMS
	robot.add_child(lidar)

	odom = OdomModel.new(Vector3(START_POS.x, START_POS.y, 0.0), ODOM_NOISE)


func _build_comms() -> void:
	ws = WSClient.new()
	ws.setup(WS_URL)
	add_child(ws)
	ws.connected.connect(_on_ws_connected)
	ws.message_received.connect(_on_message)


# ---------- 主循环 ----------

func _physics_process(dt: float) -> void:
	if mode == "manual":
		var axes: Vector2 = Teleop.read_input()
		if axes != Vector2.ZERO:
			robot.command(axes.x, axes.y)

	# 里程计积分（始终用机器人"实际指令速度"——注意指令被执行即为速度）
	odom.integrate(robot.linear, robot.angular, dt)

	# 操作员可见的真值轨迹（仅本窗口渲染，绝不上行到算法通道）
	if _truth_trail.size() == 0 or robot.global_position.distance_to(_truth_trail[_truth_trail.size() - 1]) > 6.0:
		_truth_trail.append(robot.global_position)
	queue_redraw()

	# 20Hz 上行
	_send_acc += dt
	_hz_timer += dt
	_frames_this_sec += 1
	if _hz_timer >= 1.0:
		_send_hz_meas = _frames_this_sec / _hz_timer
		_hz_timer = 0.0
		_frames_this_sec = 0
	if _send_acc >= 1.0 / SEND_HZ:
		_send_acc = 0.0
		_send_frame()


func _send_frame() -> void:
	if not ws.is_connected:
		return
	_seq += 1
	var ranges := lidar.scan()
	var odom_pose: Vector3 = odom.pose
	var true_pose := Vector3(robot.global_position.x, robot.global_position.y, robot.global_rotation)
	ws.send(Protocol.sensor_frame(_seq, odom_pose, ranges, -PI, TAU / BEAMS, RANGE_MAX, robot.collided, 1.0 / SEND_HZ))
	ws.send(Protocol.ground_truth(_seq, true_pose))


func _draw() -> void:
	# 房间
	draw_rect(Rect2(Vector2.ZERO, ROOM_SIZE), Color(0.16, 0.18, 0.22), true)
	draw_rect(Rect2(Vector2.ZERO, ROOM_SIZE), Color(0.55, 0.6, 0.7), false, 3.0)
	for rect in FURNITURE:
		draw_rect(rect, Color(0.3, 0.34, 0.42), true)
		draw_rect(rect, Color(0.5, 0.55, 0.65), false, 2.0)
	# 真值轨迹（操作员视角）
	if _truth_trail.size() > 1:
		draw_polyline(_truth_trail, Color(0.95, 0.55, 0.2, 0.8), 2.0)


# ---------- 通信事件 ----------

func _on_ws_connected() -> void:
	_seq = 0
	ws.send(Protocol.hello(0, BEAMS, RANGE_MAX, RANGE_NOISE, ODOM_NOISE))


func _on_message(msg: Dictionary) -> void:
	match msg.get("type", ""):
		"cmd_vel":
			if mode == "auto":
				var p: Dictionary = msg["payload"]
				robot.command(float(p.get("linear", 0.0)), float(p.get("angular", 0.0)))
		"set_mode":
			var m: String = str(msg["payload"].get("mode", "manual"))
			if m in ["manual", "auto"] and m != mode:
				_toggle_mode()


func _toggle_mode() -> void:
	mode = "auto" if mode == "manual" else "manual"
	robot.control_enabled = true
	if mode == "manual":
		robot.stop()
	hud.set_mode_text(mode)
	# 上行同步给 Python 端：auto = 启动探索器，manual = 停发 cmd_vel
	_mode_seq += 1
	ws.send(Protocol.set_mode(_mode_seq, mode))


func _process(_dt: float) -> void:
	if hud != null and ws != null:
		hud.update_text(
			ws.is_connected, mode, _send_hz_meas, _seq,
			Vector3(robot.global_position.x, robot.global_position.y, robot.global_rotation),
			odom.pose
		)
