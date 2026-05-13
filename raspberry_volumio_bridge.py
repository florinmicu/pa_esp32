#!/usr/bin/env python3
from __future__ import annotations

import time
from urllib.error import URLError

import serial

from bridge_core.backends.volumio import command_to_path
from bridge_core.config import env_float, env_int, env_str
from bridge_core.http_client import get_json, request_ok
from bridge_core.protocol import stat_line
from bridge_core.serial_loop import ensure_serial_open

SERIAL_PORT = env_str("SERIAL_PORT", "/dev/serial0")
SERIAL_BAUD = env_int("SERIAL_BAUD", 115200)
SERIAL_READ_TIMEOUT_S = env_float("SERIAL_READ_TIMEOUT_S", 0.1)
VOLUMIO_BASE = env_str("VOLUMIO_BASE", "http://127.0.0.1:3000")
HTTP_TIMEOUT_S = env_float("HTTP_TIMEOUT_S", 2.0)
PUSH_INTERVAL_S = env_float("STATE_PUSH_INTERVAL_S", 2.0)
REFRESH_INTERVAL_S = env_float("STATE_REFRESH_INTERVAL_S", 0.35)


def get_state_line() -> str:
    try:
        s = get_json(VOLUMIO_BASE + "/api/v1/getState", timeout_s=HTTP_TIMEOUT_S)
    except Exception:
        return stat_line("offline", "-", "-", "-")
    return stat_line(
        s.get("status", "unknown"),
        s.get("title", "-"),
        s.get("artist", "-"),
        s.get("uri", "-"),
    )


def run_command(cmd: str) -> None:
    path = command_to_path(cmd)
    if not path:
        return
    url = VOLUMIO_BASE + path
    # Volumio builds can differ: some accept POST, others react only to GET.
    try:
        request_ok(url, timeout_s=HTTP_TIMEOUT_S, method="POST")
    except Exception:
        try:
            request_ok(url, timeout_s=HTTP_TIMEOUT_S, method="GET")
        except URLError:
            pass


def main() -> None:
    ser: serial.Serial | None = None
    print("volumio bridge starting on", SERIAL_PORT)
    last_push = 0.0
    last_state_line = stat_line("unknown", "-", "-", "-")
    last_state_fetch = 0.0

    while True:
        ser = ensure_serial_open(ser, SERIAL_PORT, SERIAL_BAUD, SERIAL_READ_TIMEOUT_S)
        if ser is None:
            continue
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("CMD|"):
                run_command(line.split("|", 1)[1])
                last_state_line = get_state_line()
                last_state_fetch = time.time()
                ser.write((last_state_line + "\n").encode("utf-8"))
            elif line == "GET":
                last_state_line = get_state_line()
                last_state_fetch = time.time()
                ser.write((last_state_line + "\n").encode("utf-8"))

            now = time.time()
            if now - last_push >= PUSH_INTERVAL_S:
                last_state_line = get_state_line()
                last_state_fetch = now
                ser.write((last_state_line + "\n").encode("utf-8"))
                last_push = now
            if now - last_state_fetch >= REFRESH_INTERVAL_S:
                last_state_line = get_state_line()
                last_state_fetch = now
        except serial.SerialException as e:
            print("serial error, reopening:", e)
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(0.5)


if __name__ == "__main__":
    main()
