extends SceneTree
## 无头单测：godot --headless --script res://tests/run_tests.gd
## 覆盖 world_model 基础操作与 A* 寻路正确性。

const WorldModel = preload("res://scripts/core/world_model.gd")
const PathFinder = preload("res://scripts/core/planner/path_finder.gd")
const CoveragePlanner = preload("res://scripts/core/planner/coverage_planner.gd")

var _failed := false


func _init() -> void:
	_test_model_basics()
	_test_astar_blocked()
	_test_astar_around_wall()
	_test_coverage_plan()
	if _failed:
		print("RESULT: FAIL")
		quit(1)
	else:
		print("RESULT: PASS")
		quit(0)


func _check(cond: bool, msg: String) -> void:
	if cond:
		print("  PASS: " + msg)
	else:
		_failed = true
		print("  FAIL: " + msg)


func _test_model_basics() -> void:
	print("[world_model basics]")
	var m = WorldModel.new(10, 10, 20.0)
	m.mark_border()
	_check(m.is_free(Vector2i(5, 5)), "中心格可通行")
	_check(not m.is_free(Vector2i(0, 0)), "边界格为障碍")
	m.add_rect_px(Rect2(60, 60, 40, 40))
	_check(not m.is_free(Vector2i(3, 3)), "矩形内格子为障碍")
	m.mark_covered_at(Vector2(110, 110), 14.0)
	_check(m.is_covered(Vector2i(5, 5)), "圆心处格子被覆盖")
	_check(m.covered_ratio() > 0.0, "覆盖率大于 0")


func _test_astar_blocked() -> void:
	print("[A* blocked]")
	var m = WorldModel.new(10, 10, 20.0)
	m.mark_border()
	m.obstacle[5][5] = true
	var same := PathFinder.find_path(m, Vector2i(1, 1), Vector2i(1, 1))
	_check(same.size() == 1, "起终点相同返回单点")
	var bad := PathFinder.find_path(m, Vector2i(5, 5), Vector2i(0, 0))
	_check(bad.is_empty(), "起点为障碍返回空路径")


func _test_astar_around_wall() -> void:
	print("[A* around wall]")
	var m = WorldModel.new(10, 10, 20.0)
	m.mark_border()
	for x in range(1, 8):
		m.obstacle[5][x] = true
	var p := PathFinder.find_path(m, Vector2i(1, 1), Vector2i(1, 8))
	_check(p.size() > 0, "绕墙路径存在")
	if p.size() > 0:
		_check(p[0] == Vector2i(1, 1) and p[p.size() - 1] == Vector2i(1, 8), "路径首尾正确")
		var ok := true
		for c in p:
			if not m.is_free(c):
				ok = false
		_check(ok, "路径全部经过可通行格")


func _test_coverage_plan() -> void:
	print("[coverage plan]")
	var m = WorldModel.new(10, 10, 20.0)
	m.mark_border()
	var wp := CoveragePlanner.plan(m)
	var free := 0
	for y in 10:
		for x in 10:
			if m.is_free(Vector2i(x, y)):
				free += 1
	_check(wp.size() == free, "路点数等于可通行格数")
	if wp.size() >= 2:
		_check(wp[0] == Vector2i(8, 1) and wp[1] == Vector2i(7, 1), "首个非空行从右向左蛇形")
	if wp.size() > 10:
		var y2 := wp.filter(func(c: Vector2i) -> bool: return c.y == 2)
		_check(not y2.is_empty() and y2[0].x == 1, "次行回到从左向右")
