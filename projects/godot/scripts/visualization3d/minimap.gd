extends Control
## 2D 俯视叠加层：把轨迹/覆盖格子/机器人标记画在 3D 俯视渲染之上。
## 坐标映射：世界 (x, z) -> 叠加层 (x * scale, TITLE_H + z * scale)

const Config = preload("res://scripts/core/config.gd")

const MAP_W := 360.0
const MAP_H := 256.0
const TITLE_H := 24.0

var model
var robot: CharacterBody3D
var _trail := PackedVector2Array()


func setup(p_model, p_robot: CharacterBody3D) -> void:
	model = p_model
	robot = p_robot


func add_trail_point(p: Vector2) -> void:
	_trail.append(p)


func clear_trail() -> void:
	_trail = PackedVector2Array()


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func _process(_dt: float) -> void:
	queue_redraw()


func _draw() -> void:
	var s := MAP_W / Config.ROOM_SIZE.x
	# 标题栏与外框
	draw_rect(Rect2(0, 0, MAP_W, TITLE_H), Color(0.08, 0.09, 0.11, 0.92))
	draw_string(get_theme_default_font(), Vector2(8, 17), "2D 俯视图（轨迹/覆盖叠加）",
			HORIZONTAL_ALIGNMENT_LEFT, -1, 12, Color(0.85, 0.88, 0.92))
	draw_rect(Rect2(0.5, TITLE_H + 0.5, MAP_W - 1, MAP_H - 1), Color(0.5, 0.55, 0.6, 0.8), false, 1.0)
	if model == null:
		return
	# 覆盖格子（绿色半透明）
	var cs := Config.CELL * s
	var cov_col := Color(0.25, 0.85, 0.5, 0.4)
	for y in model.rows:
		for x in model.cols:
			if model.covered[y][x]:
				draw_rect(Rect2(x * cs, TITLE_H + y * cs, cs, cs), cov_col)
	# 家具轮廓
	for rect in Config.FURNITURE:
		draw_rect(Rect2(rect.position.x * s, TITLE_H + rect.position.y * s,
				rect.size.x * s, rect.size.y * s), Color(1, 1, 1, 0.25), false, 1.0)
	# 清扫轨迹（红色）
	if _trail.size() >= 2:
		var pts := PackedVector2Array()
		pts.resize(_trail.size())
		for i in _trail.size():
			pts[i] = Vector2(_trail[i].x * s, TITLE_H + _trail[i].y * s)
		draw_polyline(pts, Color(1.0, 0.45, 0.2, 0.95), 1.5)
	# 充电桩
	var ch: Vector2 = model.cell_center(Config.CHARGER_CELL)
	draw_rect(Rect2(ch.x * s - 5, TITLE_H + ch.y * s - 4, 10, 8), Color(0.2, 0.9, 0.7))
	# 机器人位置与朝向
	if robot != null:
		var rp := Vector2(robot.global_position.x, robot.global_position.z)
		var c := Vector2(rp.x * s, TITLE_H + rp.y * s)
		draw_circle(c, 5.0, Color(0.35, 0.65, 1.0))
		var heading: float = -robot.rotation.y
		draw_line(c, c + Vector2(cos(heading), sin(heading)) * 12.0, Color(0.9, 0.95, 1.0), 2.0)
