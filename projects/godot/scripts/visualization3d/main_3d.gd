extends Node3D
## 3D 主场景调度：组装世界/机器人/相机/俯视图/看板，驱动"传感器 → 大脑 → 指令"闭环。

const Config = preload("res://scripts/core/config.gd")
const WorldModel = preload("res://scripts/core/world_model.gd")
const RobotBrain = preload("res://scripts/core/robot_brain.gd")
const BehaviorState = preload("res://scripts/core/behavior/behavior_state.gd")
const World3DBuilder = preload("res://scripts/visualization3d/world_3d.gd")
const Robot3D = preload("res://scripts/visualization3d/robot_3d.gd")
const Minimap = preload("res://scripts/visualization3d/minimap.gd")
const Dashboard = preload("res://scripts/visualization3d/dashboard.gd")

var model
var brain
var robot
var camera: Camera3D
var minimap
var dashboard
var running := false
var overview := true
var elapsed := 0.0
var _trail := PackedVector2Array()
var _last_trail_pos := Vector2(-9999, -9999)
var _look_target := Vector3(450, 0, 240)


func _ready() -> void:
	model = WorldModel.new(Config.COLS, Config.ROWS, Config.CELL)
	model.mark_border()
	for rect in Config.FURNITURE:
		model.add_rect_px(rect)

	add_child(World3DBuilder.new())

	robot = Robot3D.new()
	robot.position = Vector3(
			(Config.START_CELL.x + 0.5) * Config.CELL, 0.0,
			(Config.START_CELL.y + 0.5) * Config.CELL)
	add_child(robot)
	brain = RobotBrain.new()
	brain.setup(model, Config.START_CELL)

	_build_camera()
	_build_minimap()
	dashboard = Dashboard.new()
	add_child(dashboard)
	dashboard.start_toggled.connect(_on_start_toggled)
	dashboard.reset_requested.connect(_on_reset)
	dashboard.view_toggle_requested.connect(_on_view_toggled)


func _build_camera() -> void:
	camera = Camera3D.new()
	camera.fov = 65.0
	add_child(camera)
	camera.current = true
	camera.global_position = Vector3(450, 680, 880)
	_look_target = Vector3(450, 0, 240)
	camera.look_at(_look_target)


## 2D 俯视图：SubViewport 正交相机自上而下渲染 3D 世界，叠加层画轨迹/覆盖
func _build_minimap() -> void:
	var ui := CanvasLayer.new()
	ui.layer = 1
	add_child(ui)

	var root := Control.new()
	root.position = Vector2(16, 16)
	root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	ui.add_child(root)

	var sv := SubViewport.new()
	sv.size = Vector2i(360, 256)
	sv.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(sv)
	var cam := Camera3D.new()
	cam.projection = Camera3D.PROJECTION_ORTHOGONAL
	cam.size = 640.0
	cam.position = Vector3(450, 800, 320)
	cam.rotation_degrees = Vector3(-90, 0, 0)
	sv.add_child(cam)
	cam.current = true

	var tex := TextureRect.new()
	tex.texture = sv.get_texture()
	tex.position = Vector2(0, 24)
	tex.size = Vector2(360, 256)
	tex.stretch_mode = TextureRect.STRETCH_SCALE
	root.add_child(tex)

	minimap = Minimap.new()
	minimap.setup(model, robot)
	minimap.size = Vector2(360, 280)
	root.add_child(minimap)


func _process(dt: float) -> void:
	var goal_pos: Vector3
	var goal_look: Vector3
	if overview:
		goal_pos = Vector3(450, 680, 880)
		goal_look = Vector3(450, 0, 240)
	else:
		var rp: Vector3 = robot.global_position
		goal_pos = rp + Vector3(0, 300, 320)
		goal_look = rp
	var k := minf(1.0, 4.0 * dt)
	_look_target = _look_target.lerp(goal_look, k)
	camera.global_position = camera.global_position.lerp(goal_pos, k)
	camera.look_at(_look_target)


func _physics_process(dt: float) -> void:
	var snapshot: Dictionary = robot.get_snapshot()
	if running:
		elapsed += dt
		var cmd: Dictionary = brain.update(snapshot, dt)
		robot.execute(cmd)
		var pos2: Vector2 = snapshot["pos"]
		model.mark_covered_at(pos2, Config.ROBOT_RADIUS)
		if pos2.distance_to(_last_trail_pos) > 6.0:
			_trail.append(pos2)
			minimap.add_trail_point(pos2)
			_last_trail_pos = pos2
		if brain.state == BehaviorState.State.DONE:
			running = false
			robot.execute({"type": "stop"})
			dashboard.set_running(false)
	var hits: Array = snapshot["lidar_hits"]
	var lidar_min: float = hits.min() if hits.size() > 0 else -1.0
	dashboard.set_status(BehaviorState.name_of(brain.state), brain.current_algorithm, running)
	dashboard.set_metrics(model.covered_ratio(), brain.battery,
			"%d/%d" % [brain.waypoints_done, brain.waypoints_total],
			brain.replans, lidar_min, snapshot["collision"], _fmt_time(elapsed))


func _on_start_toggled(p_running: bool) -> void:
	running = p_running
	if not running:
		robot.execute({"type": "stop"})


func _on_reset() -> void:
	running = false
	get_tree().reload_current_scene()


func _on_view_toggled(p_overview: bool) -> void:
	overview = p_overview


func _fmt_time(t: float) -> String:
	return "%02d:%02d" % [int(t / 60.0), int(t) % 60]
