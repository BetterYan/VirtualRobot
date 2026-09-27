extends Node3D
## 3D 房间世界构建：地板/墙体/家具/充电桩/光照环境。纯表现，不含逻辑。

const Config = preload("res://scripts/core/config.gd")

const WALL_H := 40.0
const WALL_T := 14.0

const FURNITURE_STYLE := [
	{"h": 34.0, "col": Color(0.62, 0.5, 0.38)},
	{"h": 24.0, "col": Color(0.55, 0.58, 0.62)},
	{"h": 46.0, "col": Color(0.45, 0.5, 0.42)},
	{"h": 28.0, "col": Color(0.6, 0.45, 0.4)},
]


func _ready() -> void:
	_build_environment()
	_build_light()
	_build_floor()
	_build_walls()
	_build_furniture()
	_build_charger()


func _mat(col: Color, emis := Color.TRANSPARENT) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = col
	m.roughness = 0.85
	if emis != Color.TRANSPARENT:
		m.emission_enabled = true
		m.emission = emis
		m.emission_energy_multiplier = 1.5
	return m


func _add_box(center: Vector3, size: Vector3, mat: StandardMaterial3D, with_body := true) -> void:
	var mesh := MeshInstance3D.new()
	var box := BoxMesh.new()
	box.size = size
	mesh.mesh = box
	mesh.material_override = mat
	mesh.position = center
	add_child(mesh)
	if with_body:
		var body := StaticBody3D.new()
		var shape := CollisionShape3D.new()
		var bs := BoxShape3D.new()
		bs.size = size
		shape.shape = bs
		body.position = center
		body.add_child(shape)
		add_child(body)


func _build_environment() -> void:
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.13, 0.14, 0.16)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.75, 0.77, 0.8)
	env.ambient_light_energy = 0.7
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)


func _build_light() -> void:
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55, -35, 0)
	sun.light_energy = 1.2
	sun.shadow_enabled = true
	add_child(sun)


func _build_floor() -> void:
	var half := Vector3(Config.ROOM_SIZE.x / 2.0, 0.0, Config.ROOM_SIZE.y / 2.0)
	var mesh := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Config.ROOM_SIZE
	mesh.mesh = plane
	mesh.material_override = _mat(Color(0.85, 0.8, 0.72))
	mesh.position = Vector3(half.x, 0, half.z)
	add_child(mesh)
	var body := StaticBody3D.new()
	var shape := CollisionShape3D.new()
	var bs := BoxShape3D.new()
	bs.size = Vector3(Config.ROOM_SIZE.x, 2.0, Config.ROOM_SIZE.y)
	shape.shape = bs
	body.position = Vector3(half.x, -1.0, half.z)
	body.add_child(shape)
	add_child(body)


func _build_walls() -> void:
	var w := Config.ROOM_SIZE.x
	var h := Config.ROOM_SIZE.y
	var wall_mat := _mat(Color(0.35, 0.33, 0.3))
	_add_box(Vector3(w / 2.0, WALL_H / 2.0, -WALL_T / 2.0),
			Vector3(w + WALL_T * 2, WALL_H, WALL_T), wall_mat)
	_add_box(Vector3(w / 2.0, WALL_H / 2.0, h + WALL_T / 2.0),
			Vector3(w + WALL_T * 2, WALL_H, WALL_T), wall_mat)
	_add_box(Vector3(-WALL_T / 2.0, WALL_H / 2.0, h / 2.0),
			Vector3(WALL_T, WALL_H, h), wall_mat)
	_add_box(Vector3(w + WALL_T / 2.0, WALL_H / 2.0, h / 2.0),
			Vector3(WALL_T, WALL_H, h), wall_mat)


func _build_furniture() -> void:
	for i in Config.FURNITURE.size():
		var rect: Rect2 = Config.FURNITURE[i]
		var style: Dictionary = FURNITURE_STYLE[i % FURNITURE_STYLE.size()]
		var h: float = style["h"]
		var col: Color = style["col"]
		var center := Vector3(rect.position.x + rect.size.x / 2.0, h / 2.0,
				rect.position.y + rect.size.y / 2.0)
		_add_box(center, Vector3(rect.size.x, h, rect.size.y), _mat(col))


func _build_charger() -> void:
	var c := Config.CHARGER_CELL
	var cx := (c.x + 0.5) * Config.CELL
	var cz := (c.y + 0.5) * Config.CELL
	_add_box(Vector3(cx, 3.0, cz), Vector3(24, 6, 16), _mat(Color(0.2, 0.2, 0.22)), false)
	_add_box(Vector3(cx, 6.5, cz), Vector3(16, 2, 10),
			_mat(Color(0.15, 0.6, 0.45), Color(0.1, 0.9, 0.6)), false)
