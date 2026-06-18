from dataclasses import dataclass
from typing import Any

MAGIC = bytes([0xEC, 0xB1])
VERSION = 0x01
FRAME_SIZE = 64
HEADER_SIZE = 10
CRC_OFFSET = 62
PAYLOAD_MAX = CRC_OFFSET - HEADER_SIZE

TYPE_REQUEST = 0x01
TYPE_RESPONSE = 0x81
TYPE_EVENT = 0x82

OP_PING = 0x01
OP_GET_STATUS = 0x02
OP_RUN = 0x10
OP_STOP = 0x11
OP_ESTOP = 0x12
OP_SET_SPEED_RPM = 0x13
OP_GET_CONFIG = 0x20
OP_SET_CONFIG = 0x21
OP_RESET_CONFIG = 0x22
OP_LOG_START = 0x30
OP_LOG_STOP = 0x31
OP_LOG_RATE = 0x32
OP_TELEMETRY_EVENT = 0x33

PARAM_PWM_FREQ = 0x01
PARAM_POLE_PAIRS = 0x02
PARAM_KP = 0x03
PARAM_KI = 0x04
PARAM_KD = 0x05
PARAM_MAX_SPEED = 0x06
PARAM_MIN_SPEED = 0x07
PARAM_CURRENT_LIMIT = 0x08
PARAM_TEMP_LIMIT = 0x09
PARAM_ALL = 0xFF

LOG_SPEED = 0x01
LOG_TEMP = 0x02
LOG_CURRENT = 0x03

STATUS_OK = 0x00

TYPE_NAMES = {
    TYPE_REQUEST: "request",
    TYPE_RESPONSE: "response",
    TYPE_EVENT: "event",
}

OPCODE_NAMES = {
    OP_PING: "PING",
    OP_GET_STATUS: "GET_STATUS",
    OP_RUN: "RUN",
    OP_STOP: "STOP",
    OP_ESTOP: "ESTOP",
    OP_SET_SPEED_RPM: "SET_SPEED_RPM",
    OP_GET_CONFIG: "GET_CONFIG",
    OP_SET_CONFIG: "SET_CONFIG",
    OP_RESET_CONFIG: "RESET_CONFIG",
    OP_LOG_START: "LOG_START",
    OP_LOG_STOP: "LOG_STOP",
    OP_LOG_RATE: "LOG_RATE",
    OP_TELEMETRY_EVENT: "TELEMETRY_EVENT",
}

STATUS_NAMES = {
    0x00: "OK",
    0x01: "BAD_MAGIC",
    0x02: "BAD_VERSION",
    0x03: "BAD_CRC",
    0x04: "BAD_LENGTH",
    0x05: "UNKNOWN_OPCODE",
    0x06: "UNKNOWN_PARAM",
    0x07: "INVALID_STATE",
    0x08: "UNDERLIMIT",
    0x09: "OVERLIMIT",
    0x0A: "NOT_IMPLEMENTED",
}

CONFIG_PARAM_NAMES = {
    PARAM_PWM_FREQ: "PWM_FREQ",
    PARAM_POLE_PAIRS: "POLE_PAIRS",
    PARAM_KP: "KP",
    PARAM_KI: "KI",
    PARAM_KD: "KD",
    PARAM_MAX_SPEED: "MAX_SPEED",
    PARAM_MIN_SPEED: "MIN_SPEED",
    PARAM_CURRENT_LIMIT: "CURRENT_LIMIT",
    PARAM_TEMP_LIMIT: "TEMP_LIMIT",
    PARAM_ALL: "ALL",
}

LOG_PARAM_NAMES = {
    LOG_SPEED: "SPEED",
    LOG_TEMP: "TEMP",
    LOG_CURRENT: "CURRENT",
    PARAM_ALL: "ALL",
}


def crc16_ccitt(data: bytes | bytearray) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def u16(value: int) -> bytes:
    return int(value).to_bytes(2, "little", signed=False)


def i16_centi(value: float | int) -> bytes:
    return int(round(float(value) * 100)).to_bytes(2, "little", signed=True)


def read_u16(data: bytes, offset: int = 0) -> int:
    return int.from_bytes(data[offset:offset + 2], "little", signed=False)


def read_i16(data: bytes, offset: int = 0) -> int:
    return int.from_bytes(data[offset:offset + 2], "little", signed=True)


def read_i32(data: bytes, offset: int = 0) -> int:
    return int.from_bytes(data[offset:offset + 4], "little", signed=True)


def read_u32(data: bytes, offset: int = 0) -> int:
    return int.from_bytes(data[offset:offset + 4], "little", signed=False)


def build_frame(
    seq: int,
    opcode: int,
    param: int = 0,
    payload: bytes | bytearray = b"",
    frame_type: int = TYPE_REQUEST,
    status: int = 0,
) -> bytearray:
    payload = bytes(payload)
    if len(payload) > PAYLOAD_MAX:
        raise ValueError("payload demasiado largo para un frame de 64 bytes")

    f = bytearray(FRAME_SIZE)
    f[0:2] = MAGIC
    f[2] = VERSION
    f[3] = frame_type & 0xFF
    f[4] = seq & 0xFF
    f[5] = opcode & 0xFF
    f[6] = param & 0xFF
    f[7] = status & 0xFF
    f[8:10] = u16(len(payload))
    f[10:10 + len(payload)] = payload
    crc = crc16_ccitt(f[:CRC_OFFSET])
    f[62:64] = u16(crc)
    return f


def frame(seq: int, opcode: int, param: int = 0, payload: bytes | bytearray = b"") -> bytearray:
    return build_frame(seq, opcode, param, payload)


def build_ping(seq: int = 1, payload: bytes = b"ping") -> bytearray:
    return build_frame(seq, OP_PING, payload=payload)


def build_get_status(seq: int = 1) -> bytearray:
    return build_frame(seq, OP_GET_STATUS)


def build_get_config(seq: int, param: int) -> bytearray:
    return build_frame(seq, OP_GET_CONFIG, param=param)


def build_set_speed(seq: int, rpm: int) -> bytearray:
    return build_frame(seq, OP_SET_SPEED_RPM, payload=u16(rpm))


def build_set_config(seq: int, param: int, value: int | float) -> bytearray:
    if param in (PARAM_PWM_FREQ, PARAM_MAX_SPEED, PARAM_MIN_SPEED):
        payload = u16(int(value))
    elif param == PARAM_POLE_PAIRS:
        payload = bytes([int(value) & 0xFF])
    elif param in (PARAM_KP, PARAM_KI, PARAM_KD):
        payload = i16_centi(value)
    else:
        raise ValueError(f"parametro de config no soportado en GUI: 0x{param:02X}")
    return build_frame(seq, OP_SET_CONFIG, param=param, payload=payload)


def normalize_report(data: bytes | bytearray | list[int]) -> bytes:
    data = bytes(data)
    if len(data) == FRAME_SIZE + 1 and data[0] == 0x00:
        return data[1:]
    return data


@dataclass(frozen=True)
class ParsedFrame:
    raw: bytes
    version: int
    frame_type: int
    seq: int
    opcode: int
    param: int
    status: int
    payload: bytes
    crc: int

    @property
    def type_name(self) -> str:
        return TYPE_NAMES.get(self.frame_type, f"0x{self.frame_type:02X}")

    @property
    def opcode_name(self) -> str:
        return OPCODE_NAMES.get(self.opcode, f"0x{self.opcode:02X}")

    @property
    def status_name(self) -> str:
        return STATUS_NAMES.get(self.status, f"0x{self.status:02X}")

    @property
    def param_name(self) -> str:
        if self.opcode in (OP_LOG_START, OP_LOG_STOP, OP_TELEMETRY_EVENT):
            return LOG_PARAM_NAMES.get(self.param, f"0x{self.param:02X}")
        if self.opcode in (OP_GET_CONFIG, OP_SET_CONFIG, OP_RESET_CONFIG):
            return CONFIG_PARAM_NAMES.get(self.param, f"0x{self.param:02X}")
        return f"0x{self.param:02X}"

    @property
    def ok(self) -> bool:
        return self.status == STATUS_OK

    def decode_payload(self) -> Any:
        if not self.ok:
            return None

        if self.opcode == OP_PING:
            return {
                "echo": self.payload,
                "text": _payload_text(self.payload),
            }

        if self.opcode == OP_GET_STATUS and len(self.payload) >= 10:
            return {
                "app_state": self.payload[0],
                "transport": "USB" if self.payload[1] else "UART",
                "motor_stalled": bool(self.payload[2]),
                "consistent_zero_crossing": bool(self.payload[3]),
                "speed_setpoint_rpm": read_u16(self.payload, 4),
                "actual_speed_rpm": read_u16(self.payload, 6),
                "max_pwm": read_u16(self.payload, 8),
            }

        if self.opcode == OP_GET_CONFIG:
            return decode_config_value(self.param, self.payload)

        if self.opcode == OP_TELEMETRY_EVENT and len(self.payload) >= 9:
            return {
                "variable": LOG_PARAM_NAMES.get(self.payload[0], f"0x{self.payload[0]:02X}"),
                "value": read_i32(self.payload, 1),
                "tick_ms": read_u32(self.payload, 5),
            }

        return self.payload

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": self.version,
            "type": self.frame_type,
            "type_name": self.type_name,
            "seq": self.seq,
            "opcode": self.opcode,
            "opcode_name": self.opcode_name,
            "param": self.param,
            "param_name": self.param_name,
            "status": self.status,
            "status_name": self.status_name,
            "payload_len": len(self.payload),
            "payload": self.payload,
            "payload_decoded": self.decode_payload(),
            "crc": self.crc,
        }


def _payload_text(payload: bytes) -> str | None:
    try:
        return payload.rstrip(b"\x00").decode("utf-8")
    except UnicodeDecodeError:
        return None


def decode_config_value(param: int, payload: bytes) -> Any:
    if param == PARAM_POLE_PAIRS and len(payload) >= 1:
        return payload[0]
    if param in (PARAM_PWM_FREQ, PARAM_MAX_SPEED, PARAM_MIN_SPEED) and len(payload) >= 2:
        return read_u16(payload)
    if param in (PARAM_KP, PARAM_KI, PARAM_KD) and len(payload) >= 2:
        return read_i16(payload) / 100.0
    return payload


def parse_frame(data: bytes | bytearray | list[int]) -> ParsedFrame:
    data = normalize_report(data)
    if len(data) != FRAME_SIZE:
        raise ValueError(f"frame incompleto: llegaron {len(data)} bytes")

    if data[0:2] != MAGIC:
        raise ValueError(f"magic invalido: {data[0:2].hex(' ')}")

    if data[2] != VERSION:
        raise ValueError(f"version invalida: {data[2]}")

    payload_len = read_u16(data, 8)
    if payload_len > PAYLOAD_MAX:
        raise ValueError(f"payload_len invalido: {payload_len}")

    received_crc = read_u16(data, 62)
    calculated_crc = crc16_ccitt(data[:CRC_OFFSET])
    if received_crc != calculated_crc:
        raise ValueError(
            f"crc invalido: recibido=0x{received_crc:04X}, calculado=0x{calculated_crc:04X}"
        )

    payload = data[10:10 + payload_len]
    return ParsedFrame(
        raw=data,
        version=data[2],
        frame_type=data[3],
        seq=data[4],
        opcode=data[5],
        param=data[6],
        status=data[7],
        payload=payload,
        crc=received_crc,
    )


def print_frame(parsed: ParsedFrame) -> None:
    print(
        f"seq={parsed.seq} type={parsed.type_name} opcode={parsed.opcode_name} "
        f"param={parsed.param_name} status={parsed.status_name}"
    )
    decoded = parsed.decode_payload()
    if decoded not in (None, b""):
        print(decoded)

