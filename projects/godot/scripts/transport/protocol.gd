extends RefCounted
## protocols/message-schema.md v1.0 的 Godot 实现：消息信封与编解码。
## 两端共同契约，字段改动必须先改 protocols/message-schema.md。

const VERSION := "1.0"


static func envelope(msg_type: String, seq: int, payload: Dictionary) -> Dictionary:
	return {
		"type": msg_type,
		"seq": seq,
		"ts": Time.get_unix_time_from_system() * 1000.0 as int,
		"payload": payload,
	}


## ---- 上行（Godot → Python）----

static func hello(seq: int, beams: int, range_max: float, range_noise: float,
		odom_noise: float) -> Dictionary:
	return envelope("hello", seq, {
		"world": "sim2d",
		"world_version": 1,
		"unit": "px",
		"sensor": {
			"lidar_beams": beams,
			"range_max": range_max,
			"range_noise_std": range_noise,
			"odom_noise_std": odom_noise,
		},
		"tick_hz": 60,
	})


static func sensor_frame(seq: int, odom: Vector3, ranges: PackedFloat32Array,
		angle_min: float, angle_inc: float, range_max: float,
		collision: bool, dt: float) -> Dictionary:
	return envelope("sensor_frame", seq, {
		"odom": {"x": odom.x, "y": odom.y, "theta": odom.z},
		"lidar": {
			"ranges": Array(ranges),
			"angle_min": angle_min,
			"angle_inc": angle_inc,
			"range_max": range_max,
		},
		"collision": collision,
		"dt": dt,
	})


static func ground_truth(seq: int, pose: Vector3) -> Dictionary:
	return envelope("ground_truth", seq, {"x": pose.x, "y": pose.y, "theta": pose.z})


## Godot → Python：请求切换运行模式（"manual" | "auto"）
static func set_mode(seq: int, mode: String) -> Dictionary:
	return envelope("set_mode", seq, {"mode": mode})


## ---- 下行解析（Python → Godot）----

## 返回 {"type": ..., "payload": {...}}；解析失败返回空字典。
static func parse(text: String) -> Dictionary:
	var data = JSON.parse_string(text)
	if typeof(data) != TYPE_DICTIONARY:
		return {}
	return {
		"type": str(data.get("type", "")),
		"seq": int(data.get("seq", 0)),
		"payload": data.get("payload", {}) if typeof(data.get("payload")) == TYPE_DICTIONARY else {},
	}
