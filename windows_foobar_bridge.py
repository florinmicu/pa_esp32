#!/usr/bin/env python3
"""
Minimal Windows LAN bridge for foobar2000 control.

ESP32 calls:
  http://PC_IP:8765/cmd?c=PREV
  http://PC_IP:8765/cmd?c=PLAYPAUSE
  http://PC_IP:8765/cmd?c=NEXT
  http://PC_IP:8765/state   (returns STAT|... compatible with the ESP Moode bridge)

This script controls foobar2000 via Beefweb HTTP API (no global media keys).
`/state` reads metadata from Beefweb (via `/api/query`).

Run on Windows:
  py -3 windows_foobar_bridge.py

Optional env vars (Beefweb):
  BEEFWEB_HOST (default 127.0.0.1)
  BEEFWEB_PORT (default 8880)
  BEEFWEB_USER / BEEFWEB_PASSWORD (if you enabled auth in Beefweb)
"""

from __future__ import annotations

import argparse
import base64
import json
import ctypes
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlencode, urlparse
from urllib import request

from bridge_core.backends.beefweb import is_supported_transport
from bridge_core.config import env_int, env_str
from bridge_core.protocol import normalize_player_state, sanitize_field, stat_line


VK_MEDIA_NEXT_TRACK = 0xB0
VK_MEDIA_PREV_TRACK = 0xB1
VK_MEDIA_STOP = 0xB2
VK_MEDIA_PLAY_PAUSE = 0xB3

KEYEVENTF_EXTENDEDKEY = 0x0001
KEYEVENTF_KEYUP = 0x0002

VERBOSE = False


def media_key(vk: int) -> None:
    ctypes.windll.user32.keybd_event(vk, 0, KEYEVENTF_EXTENDEDKEY, 0)
    ctypes.windll.user32.keybd_event(vk, 0, KEYEVENTF_EXTENDEDKEY | KEYEVENTF_KEYUP, 0)


BEEFWEB_HOST = env_str("BEEFWEB_HOST", "127.0.0.1")
BEEFWEB_PORT = env_int("BEEFWEB_PORT", 8880)
BEEFWEB_USER = env_str("BEEFWEB_USER", "")
BEEFWEB_PASSWORD = env_str("BEEFWEB_PASSWORD", "")


def beefweb_query_player_url() -> str:
    """
    Beefweb note (as of 0.10.x): repeating `columns=` on `/api/player` only returns the FIRST column.
    The `/api/query` endpoint supports `trcolumns` as a comma-separated list and returns all values.
    """
    cols = [
        "%title%",
        "%artist%",
        "%path%",
        # Technical fields (may be empty depending on source)
        "%bitrate%",
        "%__codec%",
        # Optional file metadata (may be unsupported on some foobar builds)
        "%file_size%",
        "%file_modified_timestamp%",
    ]
    tr = ",".join(cols)
    q = urlencode({"player": "true", "trcolumns": tr})
    return f"http://{BEEFWEB_HOST}:{BEEFWEB_PORT}/api/query?{q}"


def beefweb_open(req: request.Request):
    if BEEFWEB_USER:
        token = base64.b64encode(f"{BEEFWEB_USER}:{BEEFWEB_PASSWORD}".encode("utf-8")).decode("ascii")
        req.add_header("Authorization", f"Basic {token}")
    return request.urlopen(req, timeout=0.35)


def beefweb_control(cmd: str) -> None:
    """
    Try to control foobar via Beefweb. Raises on failure.

    Beefweb builds vary a bit across versions; we try a small set of common endpoints/methods.
    """
    cmd_u = (cmd or "").strip().upper()
    if not cmd_u:
        raise ValueError("empty cmd")

    # Normalize aliases
    if cmd_u in ("TOGGLE",):
        cmd_u = "PLAYPAUSE"

    # Candidate endpoints to try in order. Each item: (method, path, body_bytes|None, content_type|None)
    # We prefer POST with empty body when supported.
    candidates: list[tuple[str, str, bytes | None, str | None]] = []
    if cmd_u == "NEXT":
        candidates = [
            ("POST", "/api/player/next", None, None),
            ("GET", "/api/player/next", None, None),
        ]
    elif cmd_u == "PREV":
        candidates = [
            ("POST", "/api/player/previous", None, None),
            ("POST", "/api/player/prev", None, None),
            ("GET", "/api/player/previous", None, None),
            ("GET", "/api/player/prev", None, None),
        ]
    elif cmd_u in ("PLAYPAUSE", "PLAY", "PAUSE"):
        # Prefer discrete PLAY/PAUSE (does not affect other apps).
        # For PLAYPAUSE we first query current state and then issue PLAY or PAUSE.
        if cmd_u == "PLAYPAUSE":
            st_line = beefweb_state_line().strip()
            # STAT|<state>|...
            parts = st_line.split("|", 4)
            st = parts[1].strip().lower() if len(parts) >= 2 else "unknown"
            if st == "offline":
                raise RuntimeError(f"beefweb offline: {st_line}")
            cmd_u = "PAUSE" if st == "play" else "PLAY"

        if cmd_u == "PLAY":
            candidates = [
                ("POST", "/api/player/play", None, None),
                ("GET", "/api/player/play", None, None),
                # Some builds expose "start" instead of "play"
                ("POST", "/api/player/start", None, None),
                ("GET", "/api/player/start", None, None),
            ]
        else:  # PAUSE
            candidates = [
                ("POST", "/api/player/pause", None, None),
                ("GET", "/api/player/pause", None, None),
            ]

        # Fallback toggles (version-dependent)
        candidates += [
            ("POST", "/api/player/playpause", None, None),
            ("POST", "/api/player/togglepause", None, None),
            ("GET", "/api/player/playpause", None, None),
            ("GET", "/api/player/togglepause", None, None),
        ]
    elif cmd_u == "STOP":
        candidates = [
            ("POST", "/api/player/stop", None, None),
            ("GET", "/api/player/stop", None, None),
        ]
    else:
        raise ValueError("bad cmd")

    last_err: Exception | None = None
    for method, path, body, ctype in candidates:
        url = f"http://{BEEFWEB_HOST}:{BEEFWEB_PORT}{path}"
        try:
            req = request.Request(url, data=body, method=method)
            if ctype:
                req.add_header("Content-Type", ctype)
            with beefweb_open(req) as resp:
                code = getattr(resp, "status", None)
                if code is None:
                    code = getattr(resp, "code", None)
                # Consume response to allow connection reuse
                try:
                    resp.read()
                except Exception:
                    pass
            if code is None:
                # Assume ok if no code is exposed
                return
            if 200 <= int(code) < 300:
                return
            last_err = RuntimeError(f"http {code} for {method} {path}")
        except Exception as e:
            last_err = e
            continue

    raise RuntimeError(f"beefweb control failed ({cmd_u}): {last_err}")


def beefweb_state_line() -> str:
    """
    Return a line compatible with the ESP Moode bridge format:
      STAT|<state>|<title>|<artist>|<extra>
    """
    try:
        def parse_player_payload(data_obj: object) -> str | None:
            if not isinstance(data_obj, dict):
                return None
            if "error" in data_obj:
                err = str(data_obj.get("error", "error"))
                return f"STAT|offline|beefweb api|{err}|-\n"

            player_obj = data_obj.get("player", {}) if isinstance(data_obj.get("player", {}), dict) else {}
            st = normalize_player_state(player_obj.get("playbackState", "unknown"))

            active_obj = player_obj.get("activeItem", {}) if isinstance(player_obj.get("activeItem", {}), dict) else {}
            cols_local = active_obj.get("columns", [])
            if not isinstance(cols_local, list):
                cols_local = []

            def col_local(i: int) -> str:
                if i < 0 or i >= len(cols_local):
                    return ""
                return str(cols_local[i]).strip()

            # Indices must match `cols` in beefweb_query_player_url()
            tit = col_local(0) or "-"
            art = col_local(1) or "-"
            pth = col_local(2)
            br = col_local(3)
            co = col_local(4)
            fsz = col_local(5)
            fmt = col_local(6)

            tit = sanitize_field(tit, "-")
            art = sanitize_field(art, "-")
            co = sanitize_field(co, "")
            br = sanitize_field(br, "")
            pth = sanitize_field(pth, "")
            fsz = sanitize_field(fsz, "")
            fmt = sanitize_field(fmt, "")

            meta_parts_local = []
            if br and br != "?":
                meta_parts_local.append(br)
            if co and co != "?":
                meta_parts_local.append(co)
            if fsz and fsz != "?":
                meta_parts_local.append(fsz)
            if fmt and fmt != "?":
                meta_parts_local.append(f"mod:{fmt}")
            meta_local = " ".join(meta_parts_local).strip()

            file_hint_local = ""
            if pth:
                file_hint_local = pth.replace("\\", "/").split("/")[-1].strip()

            if meta_local and file_hint_local:
                extra_local = f"{meta_local} | {file_hint_local}"
            elif file_hint_local:
                extra_local = file_hint_local
            elif meta_local:
                extra_local = meta_local
            else:
                extra_local = "-"
            return stat_line(st, tit, art, extra_local) + "\n"

        url = beefweb_query_player_url()
        req = request.Request(url, method="GET")
        with beefweb_open(req) as resp:
            raw = resp.read().decode("utf-8", errors="ignore")
            code = getattr(resp, "status", None)
            if code is None:
                code = getattr(resp, "code", None)

        try:
            data = json.loads(raw)
        except Exception as e:
            head = raw.strip().replace("\r", " ").replace("\n", " ")
            if len(head) > 160:
                head = head[:160] + "..."
            return f"STAT|offline|beefweb json|{str(e)}|http {code or '-'} {head}\n"

        out = parse_player_payload(data)
        if out is not None:
            return out

        # Some builds may return the player object at the top-level instead of `{player: ...}`.
        out2 = parse_player_payload({"player": data})
        if out2 is not None:
            return out2

        return f"STAT|offline|beefweb bad|unexpected json|-\n"
    except Exception as e:
        return f"STAT|offline|beefweb net|{str(e)}|-\n"


class Handler(BaseHTTPRequestHandler):
    server_version = "foobar-bridge/1.1"

    def do_GET(self) -> None:
        u = urlparse(self.path)
        path = u.path or "/"
        if path != "/":
            path = path.rstrip("/")

        if path == "/":
            self._send(
                200,
                "windows_foobar_bridge\n"
                + "endpoints:\n"
                + "  GET  /health\n"
                + "  GET  /state\n"
                + "  GET  /cmd?c=PREV|NEXT|PLAYPAUSE|...\n"
                + f"beefweb: http://{BEEFWEB_HOST}:{BEEFWEB_PORT}/\n",
            )
            return

        if path == "/health":
            self._send(200, "ok\n")
            return
        if path == "/state":
            self._send(200, beefweb_state_line())
            return
        if path != "/cmd":
            self._send(404, "not found\n")
            return

        q = parse_qs(u.query)
        cmd = (q.get("c", [""])[0] or "").strip().upper()

        try:
            if not is_supported_transport(cmd):
                self._send(400, "bad cmd\n")
                return
            beefweb_control(cmd)
        except Exception as e:
            self._send(502, f"beefweb error: {e}\n")
            return

        self._send(200, "ok\n")

    def log_message(self, fmt: str, *args) -> None:
        if VERBOSE:
            sys.stdout.write("%s - %s\n" % (self.address_string(), fmt % args))

    def _send(self, code: int, body: str) -> None:
        b = body.encode("utf-8", errors="replace")
        self.send_response(code)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)


def main() -> int:
    p = argparse.ArgumentParser(add_help=True)
    p.add_argument("--host", default="0.0.0.0", help="Listen address (default: 0.0.0.0)")
    p.add_argument("--port", type=int, default=8765, help="Listen port (default: 8765)")
    p.add_argument("--verbose", action="store_true", help="Enable request logging and startup prints")
    # If argv contains unknown flags, argparse will exit and will NOT start the server.
    args = p.parse_args()

    host = str(args.host)
    port = int(args.port)
    global VERBOSE
    VERBOSE = bool(args.verbose)
    try:
        httpd = ThreadingHTTPServer((host, port), Handler)
    except OSError as e:
        print(f"ERROR: failed to bind {host}:{port} ({e})", file=sys.stderr)
        print("Hint: another process may already be using this port. Check:", file=sys.stderr)
        print("  netstat -ano | findstr :8765", file=sys.stderr)
        return 1
    if VERBOSE:
        print(f"foobar bridge listening on http://{host}:{port}")
        print(f"try: http://127.0.0.1:{port}/health")
        print(f"beefweb: http://{BEEFWEB_HOST}:{BEEFWEB_PORT}/ (state via http://127.0.0.1:{port}/state)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())

