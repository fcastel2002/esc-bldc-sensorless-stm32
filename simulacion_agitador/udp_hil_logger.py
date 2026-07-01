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
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


DEFAULT_BUFFER_SIZE = 4096


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


def decode_packet(data: bytes) -> str:
    return data.decode("ascii", errors="replace").strip("\0\r\n ")


def split_fields(text: str) -> list[str]:
    return [field.strip() for field in text.split(",")]


def try_int(value: str) -> int | None:
    try:
        return int(value, 10)
    except ValueError:
        return None


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


def run_listener(args: argparse.Namespace) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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
    server.bind((args.listen_host, args.listen_port))
    upstream.settimeout(args.timeout_ms / 1000.0)

    print(
        f"Proxy UDP activo: Simulink -> {args.listen_host}:{args.listen_port} -> {args.target_host}:{args.target_port}",
        flush=True,
    )

    forwarded = 0
    try:
        while args.limit <= 0 or forwarded < args.limit:
            data, remote = server.recvfrom(DEFAULT_BUFFER_SIZE)
            forwarded += 1
            write_log(make_log_entry("RX_SIMULINK", f"{remote[0]}:{remote[1]}", data), args.log_file, args.quiet)

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

            write_log(
                make_log_entry("RX_BRIDGE", f"{bridge_remote[0]}:{bridge_remote[1]}", response),
                args.log_file,
                args.quiet,
            )
            server.sendto(response, remote)
            write_log(make_log_entry("TX_SIMULINK", f"{remote[0]}:{remote[1]}", response), args.log_file, args.quiet)
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
