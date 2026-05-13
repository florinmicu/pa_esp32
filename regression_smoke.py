#!/usr/bin/env python3
from __future__ import annotations

from bridge_core.backends.moode import command_to_mpc_args
from bridge_core.backends.volumio import command_to_path
from bridge_core.protocol import normalize_player_state, stat_line


def main() -> int:
    assert normalize_player_state("playing") == "play"
    assert normalize_player_state("paused") == "pause"
    assert normalize_player_state("stopped") == "stop"
    assert stat_line("play", "A|B", "C", "D") == "STAT|play|A/B|C|D"

    assert command_to_mpc_args("PREV") == ["mpc", "prev"]
    assert command_to_mpc_args("TOGGLE") == ["mpc", "toggle"]
    assert command_to_mpc_args("VOLUME=120") == ["mpc", "volume", "100"]
    assert command_to_path("NEXT") == "/api/v1/commands/?cmd=next"
    assert command_to_path("VOLUME=12") == "/api/v1/commands/?cmd=volume&volume=12"
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

