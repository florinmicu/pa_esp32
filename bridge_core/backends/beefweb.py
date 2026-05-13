from __future__ import annotations

from bridge_core.commands import normalize_cmd


def is_supported_transport(cmd: str) -> bool:
    c = normalize_cmd(cmd)
    return c in {"PREV", "NEXT", "PLAYPAUSE", "PLAY", "PAUSE", "STOP"}

