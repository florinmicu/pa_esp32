from __future__ import annotations


def sanitize_field(value: object, default: str = "-") -> str:
    text = str(value if value is not None else "").strip()
    if not text:
        text = default
    return text.replace("|", "/")


def normalize_player_state(state: object) -> str:
    s = str(state if state is not None else "").strip().lower()
    if s in ("playing", "play"):
        return "play"
    if s in ("paused", "pause"):
        return "pause"
    if s in ("stopped", "stop"):
        return "stop"
    if s in ("offline",):
        return "offline"
    return "unknown" if not s else s


def stat_line(state: object, title: object, artist: object, extra: object) -> str:
    st = normalize_player_state(state)
    ti = sanitize_field(title, "-")
    ar = sanitize_field(artist, "-")
    ex = sanitize_field(extra, "-")
    return f"STAT|{st}|{ti}|{ar}|{ex}"

