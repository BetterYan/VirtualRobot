extends RefCounted
## 行为状态机枚举与命名

enum State { IDLE, CLEAN, DOCK, CHARGE, DONE }


static func name_of(s: int) -> String:
	var keys := State.keys()
	return keys[s] if s >= 0 and s < keys.size() else "UNKNOWN"
