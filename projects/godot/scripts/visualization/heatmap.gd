extends Node2D
## 覆盖热力图：把 world_model 的 covered 标记画成半透明绿色格子。

var model


func _draw() -> void:
	if model == null:
		return
	var cs: float = model.cell_size
	var col := Color(0.2, 0.7, 0.4, 0.35)
	for y in model.rows:
		for x in model.cols:
			if model.covered[y][x]:
				draw_rect(Rect2(x * cs, y * cs, cs, cs), col)
