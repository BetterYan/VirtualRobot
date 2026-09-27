extends RefCounted
## A* 栅格寻路（4 邻域，曼哈顿启发式）。纯静态函数，无场景依赖。


static func find_path(model, start: Vector2i, goal: Vector2i) -> Array[Vector2i]:
	var result: Array[Vector2i] = []
	if not model.is_free(start) or not model.is_free(goal):
		return result
	if start == goal:
		result.append(start)
		return result

	var came := {}          # Vector2i -> Vector2i
	var g := {start: 0}     # Vector2i -> int
	var f := {start: _h(start, goal)}
	var open: Array[Vector2i] = [start]

	while open.size() > 0:
		var best_i := 0
		for i in open.size():
			if f.get(open[i], INF) < f.get(open[best_i], INF):
				best_i = i
		var cur: Vector2i = open[best_i]
		open.remove_at(best_i)

		if cur == goal:
			var c := cur
			while c != start:
				result.append(c)
				c = came[c]
			result.append(start)
			result.reverse()
			return result

		for d in [Vector2i(1, 0), Vector2i(-1, 0), Vector2i(0, 1), Vector2i(0, -1)]:
			var nxt: Vector2i = cur + d
			if not model.is_free(nxt):
				continue
			var ng: int = g[cur] + 1
			if ng < int(g.get(nxt, 1 << 30)):
				came[nxt] = cur
				g[nxt] = ng
				f[nxt] = ng + _h(nxt, goal)
				if not open.has(nxt):
					open.append(nxt)
	return result


static func _h(a: Vector2i, b: Vector2i) -> int:
	return absi(a.x - b.x) + absi(a.y - b.y)
