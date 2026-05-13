from __future__ import annotations

import time
from typing import Optional

import serial


def ensure_serial_open(
    ser: Optional[serial.Serial],
    port: str,
    baud: int,
    timeout_s: float,
    reopen_sleep_s: float = 1.0,
) -> serial.Serial | None:
    if ser is not None and ser.is_open:
        return ser
    try:
        return serial.Serial(port, baud, timeout=timeout_s)
    except serial.SerialException:
        time.sleep(reopen_sleep_s)
        return None

