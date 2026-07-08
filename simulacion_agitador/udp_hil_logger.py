#!/usr/bin/env python3
"""Pequena herramienta para inspeccionar el trafico UDP HIL/PIL de Simulink.

Uso recomendado como proxy:

    python simulacion_agitador/udp_hil_logger.py proxy --listen-port 5057 --target-port 5055

Luego en Simulink se cambia el bloque UDP Send para enviar a `127.0.0.1:5057`.
La GUI sigue escuchando en `127.0.0.1:5055`, y este script registra y reenvia
cada datagrama para poder corroborar el contenido real del CSV.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


DEFAULT_BUFFER_SIZE = 4096
ACTIVE_SENDER_LEASE_MS = 250


def disable_udp_connreset(sock: socket.socket) -> None:
    """Evita que Windows cierre sockets UDP ante ICMP port unreachable."""
    sio_udp_connreset = getattr(socket, "SIO_UDP_CONNRESET", None)
    if sio_udp_connreset is None:
        return

    try:
        sock.ioctl(sio_udp_connreset, False)
    except OSError:
        pass


@dataclass(frozen=True)
class PacketSummary:
    raw_text: str
    kind: str
    sequence: int | None
    speed_rpm: int | None
    enable: int | None
    load_torque: int | None
    flags: int | None
    bridge_accepts: bool
    note: str | None
    fields: list[str]


@dataclass
class SenderSession:
    lease_ms: int
    active_sender: tuple[str, int] | None = None
    active_sequence: int | None = None
    last_accepted_at_ms: float = float("-inf")

    def can_accept(self, sender: tuple[str, int], summary: PacketSummary, now_ms: float) -> bool:
        if is_estop(summary):
            return True

        self._expire_if_idle(now_ms)
        if self.active_sender is None or sender == self.active_sender:
            return True

        return self.would_take_over(sender, summary, now_ms)

    def would_take_over(self, sender: tuple[str, int], summary: PacketSummary, now_ms: float) -> bool:
        self._expire_if_idle(now_ms)
        if self.active_sender is None or sender == self.active_sender:
            return False

        if is_start_command(summary):
            return True

        return (
            summary.sequence is not None
            and self.active_sequence is not None
            and summary.sequence < self.active_sequence
        )

    def accept(self, sender: tuple[str, int], summary: PacketSummary, now_ms: float) -> None:
        self.active_sender = sender
        self.active_sequence = summary.sequence
        self.last_accepted_at_ms = now_ms

    def release(self, sender: tuple[str, int] | None = None) -> None:
        if sender is not None and sender != self.active_sender:
            return

        self.active_sender = None
        self.active_sequence = None
        self.last_accepted_at_ms = float("-inf")

    def describe_active(self, now_ms: float) -> str:
        self._expire_if_idle(now_ms)
        if self.active_sender is None:
            return "none"

        return f"{self.active_sender[0]}:{self.active_sender[1]}"

    def _expire_if_idle(self, now_ms: float) -> None:
        if self.active_sender is not None and now_ms - self.last_accepted_at_ms > self.lease_ms:
            self.release()


def decode_packet(data: bytes) -> str:
    return data.decode("ascii", errors="replace").strip("\0\r\n ")


def split_fields(text: str) -> list[str]:
    return [field.strip() for field in text.split(",")]


def try_int(value: str) -> int | None:
    try:
        return int(value, 10)
    except ValueError:
        return None


def is_start_command(summary: PacketSummary) -> bool:
    return summary.kind == "command" and summary.fields and summary.fields[0].upper() in {"HIL_START", "START"}


def is_stop_command(summary: PacketSummary) -> bool:
    return summary.kind == "command" and summary.fields and summary.fields[0].upper() in {"MOTOR_STOP", "REAL_STOP", "HIL_STOP", "STOP"}


def is_run_command(summary: PacketSummary) -> bool:
    return summary.kind == "command" and summary.fields and summary.fields[0].upper() in {"RUN", "MOTOR_RUN"}


def is_estop(summary: PacketSummary) -> bool:
    return summary.kind == "command" and summary.fields and summary.fields[0].upper() == "ESTOP"


def is_forwardable_input(summary: PacketSummary) -> bool:
    return summary.kind in {"command", "setpoint", "hil_inputs", "hil_inputs_legacy"}


def summarize_packet(text: str) -> PacketSummary:
    if not text:
        return PacketSummary(text, "invalid", None, None, None, None, None, False, "paquete vacio", [])

    fields = split_fields(text)
    command = fields[0].upper()

    if command in {"RUN", "MOTOR_RUN", "MOTOR_STOP", "REAL_STOP", "HIL_START", "START", "HIL_STOP", "STOP", "ESTOP"}:
        return PacketSummary(text, "command", None, None, None, None, None, True, None, fields)

    if command in {"OK", "ERR"}:
        sequence = None
        for index in (1, 2):
            if index < len(fields):
                candidate = try_int(fields[index])
                if candidate is not None:
                    sequence = candidate
                    break

        return PacketSummary(text, "bridge_response", sequence, None, None, None, None, command == "OK", None, fields)

    if command in {"SETPOINT", "SP", "SPEED"}:
        sequence = try_int(fields[1]) if len(fields) == 3 else None
        rpm_field = fields[2] if len(fields) == 3 else fields[1] if len(fields) == 2 else ""
        rpm = try_int(rpm_field)
        accepts = rpm is not None and 0 <= rpm <= 65535 and (sequence is None or sequence >= 0)
        note = None if accepts else "el bridge exige rpm entero entre 0 y 65535"
        return PacketSummary(text, "setpoint", sequence, rpm, None, None, None, accepts, note, fields)

    offset = 1 if command in {"PIL", "HIL"} else 0
    payload_fields = fields[offset:]

    if len(payload_fields) in {2, 3}:
        has_sequence = len(payload_fields) == 3
        sequence = try_int(payload_fields[0]) if has_sequence else None
        first_value = 1 if has_sequence else 0
        speed = try_int(payload_fields[first_value])
        enable = try_int(payload_fields[first_value + 1])
        accepts = (
            (sequence is None or sequence >= 0)
            and speed is not None
            and 0 <= speed <= 65535
            and enable is not None
            and 0 <= enable <= 255
        )
        note = None
        if speed is not None and speed < 0:
            note = "velocidad negativa: el bridge la rechaza porque parsea ushort"
        elif not accepts:
            note = "formato HIL invalido para el bridge"
        return PacketSummary(text, "hil_inputs", sequence, speed, enable, None, None, accepts, note, fields)

    if len(payload_fields) in {5, 6}:
        has_sequence = len(payload_fields) == 6
        sequence = try_int(payload_fields[0]) if has_sequence else None
        first_value = 1 if has_sequence else 0
        speed = try_int(payload_fields[first_value])
        load_torque = try_int(payload_fields[first_value + 2])
        flags = try_int(payload_fields[first_value + 3])
        enable = try_int(payload_fields[first_value + 4])
        accepts = (
            (sequence is None or sequence >= 0)
            and speed is not None
            and 0 <= speed <= 65535
            and try_int(payload_fields[first_value + 1]) is not None
            and load_torque is not None
            and -32768 <= load_torque <= 32767
            and flags is not None
            and 0 <= flags <= 255
            and enable is not None
            and 0 <= enable <= 255
        )
        note = None if accepts else "formato HIL legacy invalido para el bridge"
        return PacketSummary(text, "hil_inputs_legacy", sequence, speed, enable, load_torque, flags, accepts, note, fields)

    return PacketSummary(
        text,
        "invalid",
        None,
        None,
        None,
        None,
        None,
        False,
        "no coincide con SETPOINT ni con los CSV HIL/PIL esperados",
        fields,
    )


def make_log_entry(direction: str, endpoint: str, data: bytes) -> dict[str, Any]:
    text = decode_packet(data)
    summary = summarize_packet(text)
    return {
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "direction": direction,
        "endpoint": endpoint,
        "bytes": list(data),
        "summary": asdict(summary),
    }


def write_log(entry: dict[str, Any], log_path: Path | None, quiet: bool) -> None:
    if not quiet:
        summary = entry["summary"]
        seq = summary["sequence"]
        speed = summary["speed_rpm"]
        enable = summary["enable"]
        accepts = "si" if summary["bridge_accepts"] else "no"
        note = f" note={summary['note']}" if summary["note"] else ""
        print(
            f"{entry['timestamp']} {entry['direction']} {entry['endpoint']} "
            f"kind={summary['kind']} seq={seq} speed={speed} enable={enable} accepts={accepts}{note} "
            f"raw={summary['raw_text']!r}",
            flush=True,
        )

    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(entry, ensure_ascii=True) + "\n")


def make_proxy_drop_entry(endpoint: str, data: bytes, note: str) -> dict[str, Any]:
    entry = make_log_entry("DROP_SIMULINK", endpoint, data)
    entry["summary"]["note"] = note
    return entry


def apply_sender_session(session: SenderSession, remote: tuple[str, int], summary: PacketSummary, now_ms: float) -> None:
    if is_estop(summary):
        session.release()
        return

    if is_stop_command(summary):
        session.release(remote)
        return

    if summary.kind == "hil_inputs" or summary.kind == "hil_inputs_legacy":
        if summary.enable:
            session.accept(remote, summary, now_ms)
        else:
            session.release(remote)
        return

    if is_start_command(summary) or is_run_command(summary) or summary.kind == "setpoint":
        session.accept(remote, summary, now_ms)


def run_listener(args: argparse.Namespace) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    disable_udp_connreset(sock)
    sock.bind((args.listen_host, args.listen_port))
    sock.settimeout(args.timeout_ms / 1000.0)

    print(f"Escuchando UDP en {args.listen_host}:{args.listen_port}", flush=True)

    received = 0
    try:
        while args.limit <= 0 or received < args.limit:
            try:
                data, remote = sock.recvfrom(DEFAULT_BUFFER_SIZE)
            except socket.timeout:
                continue
            except ConnectionResetError:
                continue

            received += 1
            entry = make_log_entry("RX", f"{remote[0]}:{remote[1]}", data)
            write_log(entry, args.log_file, args.quiet)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    return 0


def run_proxy(args: argparse.Namespace) -> int:
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    upstream = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender_session = SenderSession(ACTIVE_SENDER_LEASE_MS)
    disable_udp_connreset(server)
    disable_udp_connreset(upstream)
    server.bind((args.listen_host, args.listen_port))
    upstream.settimeout(args.timeout_ms / 1000.0)

    print(
        f"Proxy UDP activo: Simulink -> {args.listen_host}:{args.listen_port} -> {args.target_host}:{args.target_port}",
        flush=True,
    )

    forwarded = 0
    try:
        while args.limit <= 0 or forwarded < args.limit:
            try:
                data, remote = server.recvfrom(DEFAULT_BUFFER_SIZE)
            except ConnectionResetError:
                continue
            forwarded += 1
            endpoint = f"{remote[0]}:{remote[1]}"
            entry = make_log_entry("RX_SIMULINK", endpoint, data)
            write_log(entry, args.log_file, args.quiet)

            summary_dict = entry["summary"]
            summary = PacketSummary(
                raw_text=summary_dict["raw_text"],
                kind=summary_dict["kind"],
                sequence=summary_dict["sequence"],
                speed_rpm=summary_dict["speed_rpm"],
                enable=summary_dict["enable"],
                load_torque=summary_dict["load_torque"],
                flags=summary_dict["flags"],
                bridge_accepts=summary_dict["bridge_accepts"],
                note=summary_dict["note"],
                fields=summary_dict["fields"],
            )

            now_ms = time.monotonic() * 1000.0
            if is_forwardable_input(summary) and not sender_session.can_accept(remote, summary, now_ms):
                active_sender = sender_session.describe_active(now_ms)
                note = f"proxy descarta este sender; activo={active_sender}"
                write_log(make_proxy_drop_entry(endpoint, data, note), args.log_file, args.quiet)
                response = f"err,proxy active sender {active_sender}".encode("ascii", errors="replace")
                server.sendto(response, remote)
                write_log(make_log_entry("TX_SIMULINK", endpoint, response), args.log_file, args.quiet)
                continue

            upstream.sendto(data, (args.target_host, args.target_port))
            try:
                response, bridge_remote = upstream.recvfrom(DEFAULT_BUFFER_SIZE)
            except socket.timeout:
                timeout_entry = {
                    "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                    "direction": "TIMEOUT",
                    "endpoint": f"{args.target_host}:{args.target_port}",
                    "summary": {
                        "raw_text": "",
                        "kind": "timeout",
                        "sequence": None,
                        "speed_rpm": None,
                        "enable": None,
                        "load_torque": None,
                        "flags": None,
                        "bridge_accepts": False,
                        "note": "sin respuesta del bridge dentro del timeout",
                        "fields": [],
                    },
                }
                write_log(timeout_entry, args.log_file, args.quiet)
                continue
            except ConnectionResetError:
                timeout_entry = {
                    "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                    "direction": "RESET",
                    "endpoint": f"{args.target_host}:{args.target_port}",
                    "summary": {
                        "raw_text": "",
                        "kind": "reset",
                        "sequence": None,
                        "speed_rpm": None,
                        "enable": None,
                        "load_torque": None,
                        "flags": None,
                        "bridge_accepts": False,
                        "note": "ICMP port unreachable o reset UDP del host remoto",
                        "fields": [],
                    },
                }
                write_log(timeout_entry, args.log_file, args.quiet)
                continue

            write_log(
                make_log_entry("RX_BRIDGE", f"{bridge_remote[0]}:{bridge_remote[1]}", response),
                args.log_file,
                args.quiet,
            )
            response_text = decode_packet(response)
            if is_forwardable_input(summary) and response_text.upper().startswith("OK"):
                apply_sender_session(sender_session, remote, summary, now_ms)
            server.sendto(response, remote)
            write_log(make_log_entry("TX_SIMULINK", endpoint, response), args.log_file, args.quiet)
    except KeyboardInterrupt:
        pass
    finally:
        upstream.close()
        server.close()

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Logger/proxy UDP para HIL/PIL de Simulink.")
    subparsers = parser.add_subparsers(dest="mode", required=True)

    def add_common_arguments(common: argparse.ArgumentParser) -> None:
        common.add_argument("--listen-host", default="127.0.0.1", help="Host local donde escuchar.")
        common.add_argument("--listen-port", type=int, default=5055, help="Puerto local donde escuchar.")
        common.add_argument("--timeout-ms", type=int, default=250, help="Timeout de sockets en ms.")
        common.add_argument("--log-file", type=Path, default=None, help="Archivo JSONL opcional para guardar el log.")
        common.add_argument("--limit", type=int, default=0, help="Cantidad maxima de paquetes antes de salir. 0 = infinito.")
        common.add_argument("--quiet", action="store_true", help="No imprimir por consola; solo escribir el archivo.")

    listen_parser = subparsers.add_parser("listen", help="Escucha y registra sin reenviar.")
    add_common_arguments(listen_parser)

    proxy_parser = subparsers.add_parser("proxy", help="Escucha, registra y reenvia al bridge real.")
    add_common_arguments(proxy_parser)
    proxy_parser.set_defaults(listen_port=5057)
    proxy_parser.add_argument("--target-host", default="127.0.0.1", help="Host destino del bridge real.")
    proxy_parser.add_argument("--target-port", type=int, default=5055, help="Puerto destino del bridge real.")

    return parser


def main(argv: list[str] | None = None) -> int:
    args_list = list(sys.argv[1:] if argv is None else argv)
    if not args_list:
        args_list = ["proxy"]
    elif args_list[0] not in {"listen", "proxy", "-h", "--help"}:
        args_list = ["proxy", *args_list]

    parser = build_parser()
    args = parser.parse_args(args_list)

    if args.mode == "listen":
        return run_listener(args)
    if args.mode == "proxy":
        return run_proxy(args)

    parser.error(f"modo no soportado: {args.mode}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
