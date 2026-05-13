from __future__ import annotations

from bridge_core.commands import normalize_cmd, parse_volume


def command_to_mpc_args(cmd: str) -> list[str] | None:
    c = normalize_cmd(cmd)
    if c == "PREV":
        return ["mpc", "prev"]
    if c == "NEXT":
        return ["mpc", "next"]
    if c == "PLAYPAUSE":
        return ["mpc", "toggle"]
    if c == "PLAY":
        return ["mpc", "play"]
    if c == "PAUSE":
        return ["mpc", "pause"]
    if c == "STOP":
        return ["mpc", "stop"]
    vol = parse_volume(c)
    if vol is not None:
        return ["mpc", "volume", str(vol)]
    return None

