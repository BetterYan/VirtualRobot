extends Node
## WebSocket 客户端：连接 Python 算法端（Python 为服务端）。
## 自动重连；收发均为 JSON 文本帧（见 protocols/message-schema.md）。

const Protocol = preload("res://scripts/transport/protocol.gd")

signal connected
signal disconnected
signal message_received(msg: Dictionary)

const RECONNECT_INTERVAL := 2.0

var url := "ws://127.0.0.1:9094"
var is_connected := false

var _peer: WebSocketPeer = WebSocketPeer.new()
var _trying := false
var _reconnect_timer := 0.0
var _in_buf: PackedByteArray = PackedByteArray()


func setup(p_url: String) -> void:
	url = p_url


func connect_now() -> void:
	if _trying:
		return
	_trying = true
	_peer = WebSocketPeer.new()
	var err := _peer.connect_to_url(url)
	if err != OK:
		push_warning("WS connect error: %s" % err)
		_trying = false
		_reconnect_timer = RECONNECT_INTERVAL


func send(msg: Dictionary) -> void:
	if not is_connected:
		return
	_peer.send_text(JSON.stringify(msg))


func _process(dt: float) -> void:
	if not _trying:
		_reconnect_timer -= dt
		if _reconnect_timer <= 0.0:
			connect_now()
		return

	_peer.poll()
	var state := _peer.get_ready_state()
	match state:
		WebSocketPeer.STATE_OPEN:
			if not is_connected:
				is_connected = true
				connected.emit()
			while true:
				var pkt := _peer.get_packet()
				if pkt.size() == 0:
					break
				var text := pkt.get_string_from_utf8()
				var msg := Protocol.parse(text)
				if not msg.is_empty():
					message_received.emit(msg)
		WebSocketPeer.STATE_CLOSED:
			if is_connected:
				is_connected = false
				disconnected.emit()
			_trying = false
			_reconnect_timer = RECONNECT_INTERVAL
		_:
			pass
