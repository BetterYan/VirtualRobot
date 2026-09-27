extends Node2D
## 主场景调度：组装房间/机器人/HUD，每帧驱动"传感器 → 大脑 → 指令"闭环。

const Config = preload("res://scripts/core/config.gd")
const WorldModel = preload("res://scripts/core/world_model.gd")
const RobotBrain = preload("res://scripts/core/robot_brain.gd")
const BehaviorState = preload("res://scripts/core/behavior/behavior_state.gd")
const RobotView = preload("res://scripts/visualization/robot_view.gd")
const RoomView = preload("res://scripts/visualization/room_view.gd")
const Heatmap = preload("res://scripts/visualization/heatmap.gd")
const Hud = preload("res://scripts/visualization/hud.gd")

var model
var brain
var robot          # RobotView 实例（脚本动态类型，避免硬类型成员检查报错）
var trail: Line2D
var heatmap
var hud
var running := false
var _last_trail_pos := Vector2(-9999, -9999)


func _ready() -> void:
	_build_model()
	_build_visuals()
	_build_physics()
	_build_robot()
	_build_hud()


func _build_model() -> void:
	model = WorldModel.new(Config.COLS, Config.ROWS, Config.CELL)
	model.mark_border()
	for rect in Config.FURNITURE:
		model.add_rect_px(rect)


func _build_visuals() -> void:
	var room = RoomView.new()
	room.model = model
	add_child(room)

	heatmap = Heatmap.new()
	heatmap.model = model
	add_child(heatmap)

	trail = Line2D.new()
	trail.width = 3.0
	trail.default_color = Color(0.9, 0.35, 0.2, 0.85)
	trail.joint_mode = Line2D.LINE_JOINT_ROUND
	trail.begin_cap_mode = Line2D.LINE_CAP_ROUND
	trail.end_cap_mode = Line2D.LINE_CAP_ROUND
	add_child(trail)


func _build_physics() -> void:
	var wt := 14.0
	var w := Config.ROOM_SIZE.x
	var h := Config.ROOM_SIZE.y
	_add_static_rect(Rect2(-wt, -wt, w + wt * 2, wt))
	_add_static_rect(Rect2(-wt, h, w + wt * 2, wt))
	_add_static_rect(Rect2(-wt, 0, wt, h))
	_add_static_rect(Rect2(w, 0, wt, h))
	for rect in Config.FURNITURE:
		_add_static_rect(rect)


func _build_robot() -> void:
	robot = RobotView.new()
	robot.position = model.cell_center(Config.START_CELL)
	add_child(robot)
	brain = RobotBrain.new()
	brain.setup(model, Config.START_CELL)


func _build_hud() -> void:
	hud = Hud.new()
	add_child(hud)
	hud.start_toggled.connect(_on_start_toggled)
	hud.reset_requested.connect(_on_reset)


func _add_static_rect(rect: Rect2) -> void:
	var body := StaticBody2D.new()
	var shape := CollisionShape2D.new()
	var r := RectangleShape2D.new()
	r.size = rect.size
	shape.shape = r
	body.position = rect.get_center()
	shape.position = Vector2.ZERO
	body.add_child(shape)
	add_child(body)


func _physics_process(dt: float) -> void:
	if running:
		var snapshot: Dictionary = robot.get_snapshot()
		var cmd: Dictionary = brain.update(snapshot, dt)
		robot.execute(cmd)
		model.mark_covered_at(robot.global_position, Config.ROBOT_RADIUS)
		if robot.global_position.distance_to(_last_trail_pos) > 6.0:
			trail.add_point(robot.global_position)
			_last_trail_pos = robot.global_position
		heatmap.queue_redraw()
	hud.set_stats(BehaviorState.name_of(brain.state), model.covered_ratio(), brain.battery)


func _on_start_toggled(p_running: bool) -> void:
	running = p_running
	if not running:
		robot.execute({"type": "stop"})


func _on_reset() -> void:
	running = false
	get_tree().reload_current_scene()
