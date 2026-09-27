extends RefCounted
## 覆盖路径规划：按行蛇形（弓字形）生成全部可通行格子的遍历顺序。
## 障碍造成的"断点"由大脑层用 A* 就近衔接。


static func plan(model) -> Array[Vector2i]:
	var out: Array[Vector2i] = []
	var ltr := true
	for y in model.rows:
		if ltr:
			for x in model.cols:
				if model.is_free(Vector2i(x, y)):
					out.append(Vector2i(x, y))
		else:
			for x in range(model.cols - 1, -1, -1):
				if model.is_free(Vector2i(x, y)):
					out.append(Vector2i(x, y))
		ltr = not ltr
	return out
