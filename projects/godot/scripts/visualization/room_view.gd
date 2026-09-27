extends Node2D
## 房间渲染：地板、墙体、家具、充电桩。只画模型/配置数据，不含算法。

const Config = preload("res://scripts/core/config.gd")

var model


func _draw() -> void:
	# 地板
	draw_rect(Rect2(Vector2.ZERO, Config.ROOM_SIZE), Color(0.92, 0.9, 0.85))
	# 栅格线（淡）
	var grid_col := Color(0.0, 0.0, 0.0, 0.05)
	for x in model.cols + 1:
		var px: float = x * model.cell_size
		draw_line(Vector2(px, 0), Vector2(px, Config.ROOM_SIZE.y), grid_col, 1.0)
	for y in model.rows + 1:
		var py: float = y * model.cell_size
		draw_line(Vector2(0, py), Vector2(Config.ROOM_SIZE.x, py), grid_col, 1.0)
	# 家具
	for rect in Config.FURNITURE:
		draw_rect(rect, Color(0.55, 0.5, 0.45))
		draw_rect(rect, Color(0.4, 0.36, 0.32), false, 1.5)
	# 墙体
	var wt := 14.0
	var w := Config.ROOM_SIZE.x
	var h := Config.ROOM_SIZE.y
	draw_rect(Rect2(-wt, -wt, w + wt * 2, wt), Color(0.35, 0.33, 0.3))
	draw_rect(Rect2(-wt, h, w + wt * 2, wt), Color(0.35, 0.33, 0.3))
	draw_rect(Rect2(-wt, 0, wt, h), Color(0.35, 0.33, 0.3))
	draw_rect(Rect2(w, 0, wt, h), Color(0.35, 0.33, 0.3))
	# 充电桩
	var charger: Vector2 = model.cell_center(Config.CHARGER_CELL)
	var cs: float = model.cell_size
	draw_rect(Rect2(charger - Vector2(cs * 0.6, cs * 0.8), Vector2(cs * 1.2, cs * 1.6)),
			Color(0.1, 0.6, 0.5))
	draw_rect(Rect2(charger - Vector2(cs * 0.3, cs * 0.4), Vector2(cs * 0.6, cs * 0.5)),
			Color(0.9, 0.85, 0.3))
