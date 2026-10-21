"""调参界面的通信层：统一帧、参数表和状态解码，不包含界面或控制算法。"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import binascii
import math
import struct
import threading
import time


VERSION = 1
MAX_PAYLOAD = 128
HELLO, PARAM_INFO, PARAM_GET, PARAM_SET = 0x01, 0x02, 0x10, 0x11
STATE, STREAM, SOURCE, VISION, TELEMETRY = 0x20, 0x21, 0x30, 0x31, 0x40
ERRORS = {
    1: "不支持的操作", 2: "数据长度不正确", 3: "参数不存在、重复或字段不允许",
    4: "数值超出范围或不是有限数", 5: "遥控未在线或未进入自瞄", 6: "尚未选择串口来源",
}
STATE_FIELDS = (
    "gain", "offset_rad", "requested_angle_rad", "target_angle_rad", "actual_angle_rad",
    "speed_rad_s", "angle_error_rad", "p_out", "i_out", "d_out", "pid_unclamped_out",
    "gravity_target_out", "gravity_out", "total_unclamped_out", "total_out",
)
FLAG_FIELDS = (
    "control_enabled", "target_limited", "integral_limited", "pid_saturated", "total_saturated",
    "integral_blocked", "integral_separated", "gravity_limited", "gravity_config_invalid",
)


@dataclass(frozen=True)
class Message:
    command: int
    sequence: int
    payload: bytes = b""


def encode(message: Message) -> bytes:
    if len(message.payload) > MAX_PAYLOAD:
        raise ValueError("消息超过单帧长度")
    body = struct.pack("<BBHH", VERSION, message.command, message.sequence, len(message.payload)) + message.payload
    return b"\xa5\x5a" + body + struct.pack("<H", binascii.crc_hqx(body, 0xffff))


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.rejected = 0

    def feed(self, data: bytes) -> list[Message]:
        self.buffer.extend(data)
        messages = []
        while self.buffer:
            start = self.buffer.find(b"\xa5\x5a")
            if start < 0:
                self.buffer[:] = b"\xa5" if self.buffer[-1] == 0xa5 else b""
                break
            del self.buffer[:start]
            if len(self.buffer) < 8:
                break
            version, command, sequence, size = struct.unpack_from("<BBHH", self.buffer, 2)
            if version != VERSION or size > MAX_PAYLOAD:
                self.rejected += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < size + 10:
                break
            expected, = struct.unpack_from("<H", self.buffer, size + 8)
            if binascii.crc_hqx(self.buffer[2:size + 8], 0xffff) != expected:
                self.rejected += 1
                del self.buffer[0]
                continue
            messages.append(Message(command, sequence, bytes(self.buffer[8:size + 8])))
            del self.buffer[:size + 10]
        return messages


def decode_state(payload: bytes) -> dict:
    if len(payload) != 76 or payload[0] != 0:
        raise ValueError("状态消息长度或返回码不正确")
    values = struct.unpack("<BIBBBH15fhf", payload)
    _, tick, source, remote, automatic, flags, *tail = values
    result = dict(zip(STATE_FIELDS, tail[:15]))
    result.update(tick_ms=tick, source=source, rc_online=remote, automatic=automatic,
                  motor_cmd=tail[15], speed_ki=tail[16])
    result.update({key: bool(flags & (1 << bit)) for bit, key in enumerate(FLAG_FIELDS)})
    return result


def vision_payload(pitch_rad: float) -> bytes:
    """复用原视觉载荷及 Pitch 符号；Yaw 字段在串口测试模式下不应用。"""
    if not math.isfinite(pitch_rad) or not -math.pi <= pitch_rad <= math.pi:
        raise ValueError("Pitch 目标无效")
    # 朝零取整，避免 UI 输入角度上/下限时，量化后反而越过固件边界。
    raw = int(-pitch_rad * 32767 / math.pi)
    return struct.pack("<BhhBh", 1, 0, raw, 0, 0)


class TuningLink:
    """由界面的一个工作线程调用，避免串口等待阻塞界面。不自动重试写操作。"""
    def __init__(self, port):
        self.port = port
        self.decoder = Decoder()
        self.telemetry = deque(maxlen=1000)
        self.sequence = 0
        self.lock = threading.Lock()

    @classmethod
    def connect(cls, port_name: str):
        import serial  # 仅实物连接需要 pyserial；协议层与离线测试不需要它。
        return cls(serial.Serial(port_name, 115200, timeout=0.05, write_timeout=0.5))

    def close(self):
        self.port.close()

    def _read(self):
        return self.decoder.feed(self.port.read(max(1, min(self.port.in_waiting, 512))))

    def request(self, command: int, payload=b"", timeout=2.0) -> bytes:
        with self.lock:
            self.sequence = self.sequence % 65535 + 1
            sequence = self.sequence
            frame = encode(Message(command, sequence, payload))
            if self.port.write(frame) != len(frame):
                raise IOError("串口未完整发送；请读回确认，不自动重发写入")
            deadline = time.monotonic() + timeout
            result = None
            while time.monotonic() < deadline:
                for message in self._read():
                    if message.command == TELEMETRY:
                        self.telemetry.append(decode_state(message.payload))
                    elif message.sequence == sequence and message.command == (command | 0x80):
                        result = message.payload
                if result is not None:
                    if not result:
                        raise ValueError("设备返回空回复")
                    if result[0] != 0:
                        raise ValueError(ERRORS.get(result[0], "未知设备错误"))
                    return result
            self.decoder = Decoder()
            raise TimeoutError("设备未确认；不能据此显示已生效，请重新读回参数")

    def hello(self) -> dict:
        data = self.request(HELLO)
        if len(data) != 41 or data[1] != VERSION:
            raise ValueError("设备协议版本或握手长度不匹配")
        count, = struct.unpack_from("<H", data, 2)
        limits = struct.unpack_from("<6f", data, 4)
        return dict(protocol=data[1], parameter_count=count,
                    device=data[28:].rstrip(b"\0").decode("ascii"),
                    **dict(zip(("pitch_min_rad", "pitch_max_rad", "gain_max", "offset_max_rad",
                                "gravity_max_out", "total_max_out"), limits)))

    def parameters(self) -> list[dict]:
        result = []
        for index in range(self.hello()["parameter_count"]):
            data = self.request(PARAM_INFO, struct.pack("<H", index))
            if len(data) < 21:
                raise ValueError("参数描述长度不正确")
            identifier, unit, minimum, maximum, step, value = struct.unpack_from("<HB4f", data, 1)
            result.append(dict(id=identifier, unit=unit, minimum=minimum, maximum=maximum,
                               step=step, value=value, name=data[20:].rstrip(b"\0").decode("ascii")))
        return result

    def get_parameter(self, identifier: int) -> float:
        data = self.request(PARAM_GET, struct.pack("<H", identifier))
        if len(data) != 7 or struct.unpack_from("<H", data, 1)[0] != identifier:
            raise ValueError("参数回读不匹配")
        return struct.unpack_from("<f", data, 3)[0]

    def set_parameters(self, values: dict[int, float]) -> dict[int, float]:
        if not 1 <= len(values) <= 3:
            raise ValueError("本版每次可修改一至三个调参参数")
        payload = bytes([len(values)]) + b"".join(struct.pack("<Hf", key, value) for key, value in values.items())
        reply = self.request(PARAM_SET, payload)
        if len(reply) != len(payload) + 1 or reply[1:] != payload:
            raise ValueError("参数写入回执不匹配")
        return {key: struct.unpack("<f", struct.pack("<f", value))[0] for key, value in values.items()}

    def state(self) -> dict:
        return decode_state(self.request(STATE))

    def select_source(self, uart: bool):
        self.request(SOURCE, bytes([int(uart)]))

    def set_target(self, pitch_rad: float):
        self.request(VISION, vision_payload(pitch_rad))

    def stream(self, period_ms=200):
        self.request(STREAM, struct.pack("<H", period_ms))

    def poll(self):
        with self.lock:
            for message in self._read():
                if message.command == TELEMETRY:
                    self.telemetry.append(decode_state(message.payload))
