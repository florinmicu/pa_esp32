#!/usr/bin/env python3
import json
import os
import re
import subprocess
import sys
import time
from urllib import request

# Ensure the directory containing this script is importable (systemd WorkingDirectory may differ).
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

_BRIDGE_CORE = os.path.join(_SCRIPT_DIR, "bridge_core")
if not os.path.isdir(_BRIDGE_CORE) or not os.path.isfile(os.path.join(_BRIDGE_CORE, "__init__.py")):
    sys.stderr.write(
        "ERROR: `bridge_core` package not found next to raspberry_moode_bridge.py.\n"
        f"Expected: {_BRIDGE_CORE}/\n"
        "Fix: copy the entire `bridge_core` folder from the PC repo into the same directory as this script, e.g.\n"
        "  scp -r bridge_core user@moode:/home/flo/bridge/\n"
        "Then: sudo systemctl restart moode-bridge.service\n"
    )
    raise SystemExit(1)

import serial
from bridge_core.backends.moode import command_to_mpc_args
from bridge_core.config import env_float, env_int, env_str
from bridge_core.protocol import normalize_player_state, sanitize_field, stat_line
from bridge_core.serial_loop import ensure_serial_open


def _default_serial_port() -> str:
    # `/dev/serial0` is a udev symlink on many images; on some boots it may be missing while
    # the UART still exists as ttyAMA*. Prefer an existing device so systemd doesn't time out.
    for p in ("/dev/serial0", "/dev/ttyAMA0", "/dev/ttyAMA1", "/dev/ttyS0"):
        if os.path.exists(p):
            return p
    return "/dev/ttyAMA0"


SERIAL_PORT = env_str("SERIAL_PORT", _default_serial_port())
BAUD = env_int("SERIAL_BAUD", 115200)
# Keep ESP protocol with 4 fields, but hide actual file path by default.
HIDE_FILE_LOCATION = env_str("HIDE_FILE_LOCATION", "1") == "1"
MOODE_STATUS_URL = env_str("MOODE_STATUS_URL", "http://127.0.0.1/engine-mpd.php?cmd=get_status")
MOODE_STATUS_REFRESH_S = env_float("MOODE_STATUS_REFRESH_S", 0.8)
STATE_PUSH_INTERVAL_S = env_float("STATE_PUSH_INTERVAL_S", 1.2)
STATE_REFRESH_INTERVAL_S = env_float("STATE_REFRESH_INTERVAL_S", 2.4)
STATE_KEEPALIVE_S = env_float("STATE_KEEPALIVE_S", 4.8)
CMD_TIMEOUT_S = env_float("CMD_TIMEOUT_S", 0.7)
STATUS_CMD_TIMEOUT_S = env_float("STATUS_CMD_TIMEOUT_S", 0.35)
SERIAL_READ_TIMEOUT_S = env_float("SERIAL_READ_TIMEOUT_S", 0.02)
VIS_FIFO_NAME = env_str("VIS_FIFO_NAME", "Visualizer FIFO")
VIS_ENSURE_INTERVAL_S = env_float("VIS_ENSURE_INTERVAL_S", 20.0)
DEBUG_LOG_PATH = env_str("DEBUG_LOG_PATH", "/tmp/moode-bridge-debug.log")
_last_moode_status = {}
_last_moode_status_ts = 0.0
_last_good_title = "-"
_last_good_artist = "-"
_last_good_uri = "-"
_last_good_state = "unknown"
_last_vis_check_ts = 0.0


def run_cmd(args, timeout_s=None):
    if timeout_s is None:
        timeout_s = CMD_TIMEOUT_S
    return subprocess.run(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        timeout=timeout_s,
        check=False,
    )


def debug_log(msg: str) -> None:
    """Append-only debug log (works even when systemd doesn't show python prints)."""
    try:
        ts = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())
        with open(DEBUG_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(f"{ts} {msg}\n")
    except Exception:
        pass


def get_moode_status(force=False):
    global _last_moode_status, _last_moode_status_ts
    now = time.time()
    if (not force) and _last_moode_status and (now - _last_moode_status_ts) < MOODE_STATUS_REFRESH_S:
        return _last_moode_status
    try:
        with request.urlopen(MOODE_STATUS_URL, timeout=0.35) as resp:
            data = json.loads(resp.read().decode("utf-8", errors="ignore"))
            if isinstance(data, dict) and data:
                _last_moode_status = data
                _last_moode_status_ts = now
                return data
    except Exception:
        pass
    return _last_moode_status if _last_moode_status else {}


def get_state_line(force_moode_refresh=False):
    global _last_good_title, _last_good_artist, _last_good_uri, _last_good_state
    try:
        moode = get_moode_status(force=force_moode_refresh)
        status_proc = run_cmd(["mpc", "status"], timeout_s=STATUS_CMD_TIMEOUT_S)
        status_text = status_proc.stdout.strip()
        status_low = status_text.lower()

        state = normalize_player_state(moode.get("state", "unknown"))
        # Prefer transport state from MPD: it flips immediately on PLAY/PAUSE.
        if "[playing]" in status_low:
            state = "play"
        elif "[paused]" in status_low:
            state = "pause"
        elif "[stopped]" in status_low:
            state = "stop"
        elif status_proc.returncode != 0 or not status_text:
            state = "offline"

        title = str(moode.get("title", "")).strip() or "-"
        artist = str(moode.get("artist", "")).strip() or "-"
        stream_name = str(moode.get("name", "")).strip()
        uri = str(moode.get("file", "")).strip() or "-"
        if artist == "-" and stream_name:
            artist = stream_name

        # Read tags independently to avoid delimiter/parsing issues from combined templates.
        m_title = run_cmd(["mpc", "-f", "%title%", "current"], timeout_s=STATUS_CMD_TIMEOUT_S).stdout.strip()
        m_artist = run_cmd(["mpc", "-f", "%artist%", "current"], timeout_s=STATUS_CMD_TIMEOUT_S).stdout.strip()
        m_name = run_cmd(["mpc", "-f", "%name%", "current"], timeout_s=STATUS_CMD_TIMEOUT_S).stdout.strip()
        m_file = run_cmd(["mpc", "-f", "%file%", "current"], timeout_s=STATUS_CMD_TIMEOUT_S).stdout.strip()
        if m_title:
            title = m_title
        if m_artist:
            artist = m_artist
        elif artist == "-" and m_name:
            artist = m_name
        if m_file:
            uri = m_file

        khz = ""
        bit_depth = ""
        kbps = ""
        m_sr = moode.get("audio_sample_rate", None)
        m_bd = moode.get("audio_sample_depth", None)
        m_bitrate = str(moode.get("bitrate", "")).strip()
        m_fmt = str(moode.get("audio_format", "")).strip()
        m_encoded = str(moode.get("encoded", "")).strip()

        if m_sr not in (None, "", "0"):
            try:
                sr_raw = float(m_sr)
                sr_k = (sr_raw / 1000.0) if sr_raw > 2000 else sr_raw
                khz = f"{sr_k:.1f}".rstrip("0").rstrip(".")
            except Exception:
                pass
        if m_bd not in (None, "", "0"):
            try:
                bit_depth = str(int(float(m_bd)))
            except Exception:
                pass
        m_bitrate_k = re.search(r"(\d+(?:\.\d+)?)\s*kbps", m_bitrate, re.IGNORECASE)
        m_bitrate_m = re.search(r"(\d+(?:\.\d+)?)\s*mbps", m_bitrate, re.IGNORECASE)
        if m_bitrate_k:
            try:
                kbps = str(int(float(m_bitrate_k.group(1))))
            except Exception:
                kbps = m_bitrate_k.group(1)
        elif m_bitrate_m:
            try:
                kbps = str(int(float(m_bitrate_m.group(1)) * 1000.0))
            except Exception:
                pass

        codec = "-"
        m_codec = re.search(r"\b(FLAC|MP3|AAC|OGG|WAV|ALAC|DSD|AIFF)\b", m_encoded, re.IGNORECASE)
        if m_codec:
            codec = m_codec.group(1).upper()
        elif m_fmt:
            codec = m_fmt.upper()
        src = uri.lower()
        if codec == "-" and ".flac" in src:
            codec = "FLAC"
        elif codec == "-" and (".aac" in src or "aac" in src):
            codec = "AAC"
        elif codec == "-" and (".mp3" in src or "mp3" in src):
            codec = "MP3"
        elif codec == "-" and (".ogg" in src or "ogg" in src):
            codec = "OGG"
        elif codec == "-" and ".wav" in src:
            codec = "WAV"

        # fallback only when moode status misses format values
        if not khz and not bit_depth and not kbps:
            status_verbose = run_cmd(["mpc", "--verbose", "status"], timeout_s=STATUS_CMD_TIMEOUT_S).stdout.lower()
            m_khz = re.search(r"(\d+(?:\.\d+)?)\s*khz", status_verbose, re.IGNORECASE)
            if m_khz:
                khz = m_khz.group(1)
            m_bit = re.search(r"(\d+)\s*bit", status_verbose, re.IGNORECASE)
            if m_bit:
                bit_depth = m_bit.group(1)
            m_kbps = re.search(r"(\d+)\s*kbps", status_verbose, re.IGNORECASE)
            if m_kbps:
                kbps = m_kbps.group(1)

        fmt_parts = []
        if khz:
            fmt_parts.append(khz)
        if bit_depth:
            fmt_parts.append(bit_depth)
        elif kbps:
            fmt_parts.append(f"{kbps}k")
        if codec != "-":
            fmt_parts.append(codec)
        fmt_info = "/".join(fmt_parts) if fmt_parts else codec
        if not fmt_info or fmt_info == "-":
            fmt_info = "-"

        title = sanitize_field(title, "-")
        artist = sanitize_field(artist, "-")
        if HIDE_FILE_LOCATION:
            uri_out = fmt_info
        else:
            uri_out = uri if uri != "-" else fmt_info
        uri_out = sanitize_field(uri_out, "-")
        if title and title != "-":
            _last_good_title = title
        if artist and artist != "-":
            _last_good_artist = artist
        if uri_out and uri_out != "-":
            _last_good_uri = uri_out
        if state and state != "unknown":
            _last_good_state = state
        return stat_line(state, title, artist, uri_out)
    except Exception:
        return stat_line("offline", "-", "-", "-")


def get_state_line_quick(force_moode_refresh=False):
    """Low-latency state line for periodic refresh (no mpc subprocess calls)."""
    global _last_good_title, _last_good_artist, _last_good_uri, _last_good_state
    try:
        moode = get_moode_status(force=force_moode_refresh)
        if not moode:
            return f"STAT|{_last_good_state}|{_last_good_title}|{_last_good_artist}|{_last_good_uri}"

        state = normalize_player_state(moode.get("state", "unknown"))

        title = str(moode.get("title", "")).strip() or "-"
        artist = str(moode.get("artist", "")).strip() or "-"
        stream_name = str(moode.get("name", "")).strip()
        uri = str(moode.get("file", "")).strip() or "-"
        if artist == "-" and stream_name:
            artist = stream_name

        khz = ""
        bit_depth = ""
        kbps = ""
        m_sr = moode.get("audio_sample_rate", None)
        m_bd = moode.get("audio_sample_depth", None)
        m_bitrate = str(moode.get("bitrate", "")).strip()
        m_fmt = str(moode.get("audio_format", "")).strip()
        m_encoded = str(moode.get("encoded", "")).strip()

        if m_sr not in (None, "", "0"):
            try:
                sr_raw = float(m_sr)
                sr_k = (sr_raw / 1000.0) if sr_raw > 2000 else sr_raw
                khz = f"{sr_k:.1f}".rstrip("0").rstrip(".")
            except Exception:
                pass
        if m_bd not in (None, "", "0"):
            try:
                bit_depth = str(int(float(m_bd)))
            except Exception:
                pass
        m_bitrate_k = re.search(r"(\d+(?:\.\d+)?)\s*kbps", m_bitrate, re.IGNORECASE)
        m_bitrate_m = re.search(r"(\d+(?:\.\d+)?)\s*mbps", m_bitrate, re.IGNORECASE)
        if m_bitrate_k:
            try:
                kbps = str(int(float(m_bitrate_k.group(1))))
            except Exception:
                kbps = m_bitrate_k.group(1)
        elif m_bitrate_m:
            try:
                kbps = str(int(float(m_bitrate_m.group(1)) * 1000.0))
            except Exception:
                pass

        codec = "-"
        m_codec = re.search(r"\b(FLAC|MP3|AAC|OGG|WAV|ALAC|DSD|AIFF)\b", m_encoded, re.IGNORECASE)
        if m_codec:
            codec = m_codec.group(1).upper()
        elif m_fmt:
            codec = m_fmt.upper()

        fmt_parts = []
        if khz:
            fmt_parts.append(khz)
        if bit_depth:
            fmt_parts.append(bit_depth)
        elif kbps:
            fmt_parts.append(f"{kbps}k")
        if codec != "-":
            fmt_parts.append(codec)
        fmt_info = "/".join(fmt_parts) if fmt_parts else codec
        if not fmt_info or fmt_info == "-":
            fmt_info = "-"

        title = sanitize_field(title, "-")
        artist = sanitize_field(artist, "-")
        uri_out = fmt_info if HIDE_FILE_LOCATION else (uri if uri != "-" else fmt_info)
        uri_out = sanitize_field(uri_out, "-")
        if title == "-" and _last_good_title != "-":
            title = _last_good_title
        else:
            _last_good_title = title
        if artist == "-" and _last_good_artist != "-":
            artist = _last_good_artist
        else:
            _last_good_artist = artist
        if uri_out == "-" and _last_good_uri != "-":
            uri_out = _last_good_uri
        else:
            _last_good_uri = uri_out
        if state == "unknown" and _last_good_state != "unknown":
            state = _last_good_state
        else:
            _last_good_state = state
        return stat_line(state, title, artist, uri_out)
    except Exception:
        return stat_line(_last_good_state, _last_good_title, _last_good_artist, _last_good_uri)


def run_command(cmd: str):
    args = command_to_mpc_args(cmd)
    if args:
        run_cmd(args)


def ensure_visualizer_output_enabled(force=False):
    global _last_vis_check_ts
    now = time.time()
    if (not force) and (now - _last_vis_check_ts) < VIS_ENSURE_INTERVAL_S:
        return
    _last_vis_check_ts = now
    try:
        out = run_cmd(["mpc", "outputs"], timeout_s=STATUS_CMD_TIMEOUT_S)
        text = out.stdout
        if out.returncode != 0 or not text:
            return
        target_id = None
        target_enabled = False
        current_id = None
        seen = []
        for raw in text.splitlines():
            line = raw.strip()
            m_out = re.match(r"^Output\s+(\d+)\s+\((.*)\)\s+is\s+(enabled|disabled)$", line, re.IGNORECASE)
            if m_out:
                current_id = m_out.group(1)
                name = m_out.group(2).strip()
                state = m_out.group(3).strip().lower()
                seen.append((current_id, name, state))
                if name == VIS_FIFO_NAME:
                    target_id = current_id
                    target_enabled = (state == "enabled")
                continue
        if target_id and not target_enabled:
            run_cmd(["mpc", "enable", str(target_id)], timeout_s=STATUS_CMD_TIMEOUT_S)
        elif not target_id and force and seen:
            # Helpful diagnostics when the configured VIS_FIFO_NAME does not exist.
            print(f"[vis] output '{VIS_FIFO_NAME}' not found. Available outputs:")
            for oid, name, state in seen:
                print(f"[vis] - Output {oid} ({name}) is {state}")
    except Exception:
        pass


def main():
    ser: serial.Serial | None = None
    print("moode bridge starting on", SERIAL_PORT)
    last_push = 0.0
    last_state_tx = 0.0
    last_state_refresh = 0.0
    last_sent_state_line = ""
    state_line_cache = "STAT|unknown|-|-|-"
    burst_until = 0.0

    while True:
        was_open = (ser is not None and ser.is_open)
        ser = ensure_serial_open(ser, SERIAL_PORT, BAUD, SERIAL_READ_TIMEOUT_S)
        if ser is None:
            continue
        if not was_open:
            print("moode bridge connected on", SERIAL_PORT)
        try:
            ensure_visualizer_output_enabled(False)
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("CMD|"):
                try:
                    run_command(line.split("|", 1)[1])
                except Exception:
                    pass
                # Force fresh metadata right after control commands.
                state_line_cache = get_state_line(force_moode_refresh=True)
                last_state_refresh = time.time()
                ser.write((state_line_cache + "\n").encode("utf-8"))
                last_sent_state_line = state_line_cache
                last_state_tx = time.time()
                ensure_visualizer_output_enabled(True)
                burst_until = time.time() + 2.0
            elif line == "GET":
                if (time.time() - last_state_refresh) > 0.7:
                    state_line_cache = get_state_line()
                    last_state_refresh = time.time()
                ser.write((state_line_cache + "\n").encode("utf-8"))
                last_sent_state_line = state_line_cache
                last_state_tx = time.time()

            now = time.time()
            refresh_period = 0.35 if now < burst_until else STATE_REFRESH_INTERVAL_S
            if now - last_state_refresh >= refresh_period:
                state_line_cache = get_state_line(force_moode_refresh=(now < burst_until))
                last_state_refresh = now

            push_period = 0.35 if now < burst_until else STATE_PUSH_INTERVAL_S
            if now - last_push >= push_period:
                needs_send = (state_line_cache != last_sent_state_line) or ((now - last_state_tx) >= STATE_KEEPALIVE_S)
                if needs_send:
                    ser.write((state_line_cache + "\n").encode("utf-8"))
                    last_sent_state_line = state_line_cache
                    last_state_tx = now
                last_push = now
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
