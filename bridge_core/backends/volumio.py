from __future__ import annotations

from bridge_core.commands import normalize_cmd, parse_volume


def command_to_path(cmd: str) -> str | None:
    c = normalize_cmd(cmd)
    if c == "PREV":
        return "/api/v1/commands/?cmd=prev"
    if c == "NEXT":
        return "/api/v1/commands/?cmd=next"
    if c == "PLAYPAUSE":
        return "/api/v1/commands/?cmd=toggle"
    if c == "PLAY":
        return "/api/v1/commands/?cmd=play"
    if c == "PAUSE":
        return "/api/v1/commands/?cmd=pause"
    if c == "STOP":
        return "/api/v1/commands/?cmd=stop"
    vol = parse_volume(c)
    if vol is not None:
        return f"/api/v1/commands/?cmd=volume&volume={vol}"
    return None

