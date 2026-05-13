from __future__ import annotations

from typing import Callable


CommandHandler = Callable[[str], None]


def normalize_cmd(cmd: str) -> str:
    c = (cmd or "").strip().upper()
    if c == "TOGGLE":
        return "PLAYPAUSE"
    return c


def parse_volume(cmd: str) -> int | None:
    c = normalize_cmd(cmd)
    if not c.startswith("VOLUME="):
        return None
    try:
        value = int(c.split("=", 1)[1])
    except Exception:
        return None
    return max(0, min(100, value))


def dispatch_basic_transport(cmd: str, handlers: dict[str, CommandHandler]) -> bool:
    c = normalize_cmd(cmd)
    h = handlers.get(c)
    if h is None:
        return False
    h(c)
    return True

