extends RefCounted
## 栅格世界模型：障碍层 + 覆盖标记层（机器人对世界的"认知"，归逻辑层所有）
## 纯逻辑类，禁止引用任何渲染节点。

var cols: int
var rows: int
var cell_size: float
var obstacle: Array = []   # obstacle[y][x] -> bool
var covered: Array = []    # covered[y][x]  -> bool

var _free_count := 0


func _init(p_cols: int, p_rows: int, p_cell: float) -> void:
	cols = p_cols
	rows = p_rows
	cell_size = p_cell
	for y in rows:
		var o_row := []
		var c_row := []
		for x in cols:
			o_row.append(false)
			c_row.append(false)
		obstacle.append(o_row)
		covered.append(c_row)


func in_bounds(c: Vector2i) -> bool:
	return c.x >= 0 and c.x < cols and c.y >= 0 and c.y < rows


func is_free(c: Vector2i) -> bool:
	return in_bounds(c) and not obstacle[c.y][c.x]


func is_covered(c: Vector2i) -> bool:
	return in_bounds(c) and covered[c.y][c.x]


## 将一个像素矩形内的格子标记为障碍（家具）
func add_rect_px(rect: Rect2) -> void:
	var x0 := int(floor(rect.position.x / cell_size))
	var y0 := int(floor(rect.position.y / cell_size))
	var x1 := int(ceil(rect.end.x / cell_size))
	var y1 := int(ceil(rect.end.y / cell_size))
	for y in range(y0, y1):
		for x in range(x0, x1):
			var center := Vector2((x + 0.5) * cell_size, (y + 0.5) * cell_size)
			if in_bounds(Vector2i(x, y)) and rect.has_point(center):
				obstacle[y][x] = true


## 标记外圈边界为障碍（墙体）
func mark_border() -> void:
	for y in rows:
		obstacle[y][0] = true
		obstacle[y][cols - 1] = true
	for x in cols:
		obstacle[0][x] = true
		obstacle[rows - 1][x] = true


func _recalc_free() -> void:
	_free_count = 0
	for y in rows:
		for x in cols:
			if not obstacle[y][x]:
				_free_count += 1


func free_count() -> int:
	if _free_count == 0:
		_recalc_free()
	return _free_count


## 以像素坐标 pos 为圆心、radius_px 为半径，标记已清扫格子，返回新增数量
func mark_covered_at(pos: Vector2, radius_px: float) -> int:
	var added := 0
	var c0 := cell_of(pos - Vector2(radius_px, radius_px))
	var c1 := cell_of(pos + Vector2(radius_px, radius_px))
	for y in range(maxi(c0.y, 0), mini(c1.y + 1, rows)):
		for x in range(maxi(c0.x, 0), mini(c1.x + 1, cols)):
			if obstacle[y][x]:
				continue
			var center := Vector2((x + 0.5) * cell_size, (y + 0.5) * cell_size)
			if center.distance_to(pos) <= radius_px and not covered[y][x]:
				covered[y][x] = true
				added += 1
	return added


func covered_ratio() -> float:
	var cov := 0
	for y in rows:
		for x in cols:
			if covered[y][x]:
				cov += 1
	var total := free_count()
	return 0.0 if total == 0 else float(cov) / float(total)


func cell_of(pos: Vector2) -> Vector2i:
	return Vector2i(int(pos.x / cell_size), int(pos.y / cell_size))


func cell_center(c: Vector2i) -> Vector2:
	return Vector2((c.x + 0.5) * cell_size, (c.y + 0.5) * cell_size)
