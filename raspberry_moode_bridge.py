#!/usr/bin/env python3
import json
import os
import random
import re
import subprocess
import tempfile
import threading
import time
from urllib import request

import serial

SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/serial0")
BAUD = int(os.getenv("SERIAL_BAUD", "115200"))
FFT_BARS = 12
ENABLE_FFT = os.getenv("ENABLE_FFT", "1") == "1"
FFT_PUSH_INTERVAL_S = float(os.getenv("FFT_PUSH_INTERVAL_S", "0.08"))
CAVA_FRAMERATE = int(os.getenv("CAVA_FRAMERATE", "35"))
CAVA_ALSA_SOURCE = os.getenv("CAVA_ALSA_SOURCE", "plughw:Loopback,1,0")
CAVA_INPUT_MODE = os.getenv("CAVA_INPUT_MODE", "fifo")  # fifo | alsa
CAVA_FIFO_PATH = os.getenv("CAVA_FIFO_PATH", "/tmp/mpd.fifo")
USE_PROXY_FFT = os.getenv("USE_PROXY_FFT", "0") == "1"
CAVA_BIN = os.getenv("CAVA_BIN", "cava")
# Stereo frame parsing mode for cava raw output:
# - auto: evaluate both layouts and keep the better-separated one
# - interleaved: L0,R0,L1,R1,...
# - block: L0..L(n-1),R0..R(n-1)
CAVA_STEREO_LAYOUT = os.getenv("CAVA_STEREO_LAYOUT", "auto").strip().lower()
# Keep ESP protocol with 4 fields, but hide actual file path by default.
HIDE_FILE_LOCATION = os.getenv("HIDE_FILE_LOCATION", "1") == "1"
MOODE_STATUS_URL = os.getenv("MOODE_STATUS_URL", "http://127.0.0.1/engine-mpd.php?cmd=get_status")
MOODE_STATUS_REFRESH_S = float(os.getenv("MOODE_STATUS_REFRESH_S", "0.8"))
STATE_PUSH_INTERVAL_S = float(os.getenv("STATE_PUSH_INTERVAL_S", "1.2"))
STATE_REFRESH_INTERVAL_S = float(os.getenv("STATE_REFRESH_INTERVAL_S", "2.4"))
STATE_KEEPALIVE_S = float(os.getenv("STATE_KEEPALIVE_S", "4.8"))
CMD_TIMEOUT_S = float(os.getenv("CMD_TIMEOUT_S", "0.7"))
STATUS_CMD_TIMEOUT_S = float(os.getenv("STATUS_CMD_TIMEOUT_S", "0.35"))
SERIAL_READ_TIMEOUT_S = float(os.getenv("SERIAL_READ_TIMEOUT_S", "0.02"))
VIS_FIFO_NAME = os.getenv("VIS_FIFO_NAME", "Visualizer FIFO")
VIS_ENSURE_INTERVAL_S = float(os.getenv("VIS_ENSURE_INTERVAL_S", "20.0"))
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

        state = str(moode.get("state", "unknown")).strip().lower()
        if state == "playing":
            state = "play"
        elif state == "paused":
            state = "pause"
        elif state == "stopped":
            state = "stop"
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

        title = title.replace("|", "/")
        artist = artist.replace("|", "/")
        if HIDE_FILE_LOCATION:
            uri_out = fmt_info
        else:
            uri_out = uri if uri != "-" else fmt_info
        uri_out = uri_out.replace("|", "/")
        if title and title != "-":
            _last_good_title = title
        if artist and artist != "-":
            _last_good_artist = artist
        if uri_out and uri_out != "-":
            _last_good_uri = uri_out
        if state and state != "unknown":
            _last_good_state = state
        return f"STAT|{state}|{title}|{artist}|{uri_out}"
    except Exception:
        return "STAT|offline|-|-|-"


def get_state_line_quick(force_moode_refresh=False):
    """Low-latency state line for periodic refresh (no mpc subprocess calls)."""
    global _last_good_title, _last_good_artist, _last_good_uri, _last_good_state
    try:
        moode = get_moode_status(force=force_moode_refresh)
        if not moode:
            return f"STAT|{_last_good_state}|{_last_good_title}|{_last_good_artist}|{_last_good_uri}"

        state = str(moode.get("state", "unknown")).strip().lower()
        if state == "playing":
            state = "play"
        elif state == "paused":
            state = "pause"
        elif state == "stopped":
            state = "stop"

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

        title = title.replace("|", "/")
        artist = artist.replace("|", "/")
        uri_out = fmt_info if HIDE_FILE_LOCATION else (uri if uri != "-" else fmt_info)
        uri_out = uri_out.replace("|", "/")
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
        return f"STAT|{state}|{title}|{artist}|{uri_out}"
    except Exception:
        return f"STAT|{_last_good_state}|{_last_good_title}|{_last_good_artist}|{_last_good_uri}"


def run_command(cmd: str):
    cmd = cmd.strip().upper()
    if cmd == "PREV":
        run_cmd(["mpc", "prev"])
    elif cmd == "NEXT":
        run_cmd(["mpc", "next"])
    elif cmd in ("PLAYPAUSE", "TOGGLE"):
        run_cmd(["mpc", "toggle"])
    elif cmd == "PLAY":
        run_cmd(["mpc", "play"])
    elif cmd == "PAUSE":
        run_cmd(["mpc", "pause"])
    elif cmd == "STOP":
        run_cmd(["mpc", "stop"])
    elif cmd.startswith("VOLUME="):
        try:
            vol = int(cmd.split("=", 1)[1])
            vol = max(0, min(100, vol))
            run_cmd(["mpc", "volume", str(vol)])
        except Exception:
            pass


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
        for raw in text.splitlines():
            line = raw.strip()
            m_out = re.match(r"^Output\s+(\d+)\s+\((.*)\)\s+is\s+(enabled|disabled)$", line, re.IGNORECASE)
            if m_out:
                current_id = m_out.group(1)
                name = m_out.group(2).strip()
                state = m_out.group(3).strip().lower()
                if name == VIS_FIFO_NAME:
                    target_id = current_id
                    target_enabled = (state == "enabled")
                continue
        if target_id and not target_enabled:
            run_cmd(["mpc", "enable", str(target_id)], timeout_s=STATUS_CMD_TIMEOUT_S)
    except Exception:
        pass


class CavaBridge:
    def __init__(self, bars: int):
        self.bars = bars
        self.left = [0] * bars
        self.right = [0] * bars
        self.lock = threading.Lock()
        self.proc = None
        self.cfg_path = None
        self.thread = None
        self.frames = 0
        self.last_frame_ts = 0.0
        self._auto_votes_inter = 0
        self._auto_votes_block = 0
        self._locked_layout = None

    def start(self):
        if CAVA_INPUT_MODE == "fifo":
            if not os.path.exists(CAVA_FIFO_PATH):
                os.mkfifo(CAVA_FIFO_PATH)
            # MPD often runs as user "mpd"; keep FIFO world-writable so MPD can open it.
            # Without this, FIFO may end up 0644 (root-owned) and FFT receives no audio.
            try:
                os.chmod(CAVA_FIFO_PATH, 0o666)
            except Exception:
                pass
            input_block = f"""
[input]
method = fifo
source = {CAVA_FIFO_PATH}
sample_rate = 44100
sample_bits = 16
channels = 2
""".strip()
        else:
            input_block = f"""
[input]
method = alsa
source = {CAVA_ALSA_SOURCE}
""".strip()

        cfg = f"""
[general]
bars = {self.bars}
framerate = {CAVA_FRAMERATE}
sensitivity = 100
autosens = 1

{input_block}

[output]
method = raw
raw_target = /dev/stdout
bit_format = 8bit
channels = stereo
""".strip()
        fd, self.cfg_path = tempfile.mkstemp(prefix="cava_", suffix=".conf")
        os.close(fd)
        with open(self.cfg_path, "w", encoding="utf-8") as f:
            f.write(cfg + "\n")

        try:
            self.proc = subprocess.Popen(
                [CAVA_BIN, "-p", self.cfg_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except Exception:
            self.proc = None
            return False

        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        try:
            if self.proc is not None:
                self.proc.terminate()
                self.proc.wait(timeout=0.5)
        except Exception:
            pass
        try:
            if self.proc is not None and self.proc.poll() is None:
                self.proc.kill()
        except Exception:
            pass
        self.proc = None

    def _reader_loop(self):
        if not self.proc or not self.proc.stdout:
            return
        frame_size = self.bars * 2
        buf = bytearray()

        def _stretch_to_bars(vals, out_n):
            if not vals:
                return [0] * out_n
            if len(vals) == out_n:
                return vals[:]
            if len(vals) == 1:
                return [vals[0]] * out_n
            out = []
            src_n = len(vals)
            for i in range(out_n):
                pos = i * (src_n - 1) / (out_n - 1)
                lo = int(pos)
                hi = min(src_n - 1, lo + 1)
                frac = pos - lo
                v = int(vals[lo] * (1.0 - frac) + vals[hi] * frac)
                out.append(max(0, min(255, v)))
            return out

        def _dedup_if_needed(v):
            n = len(v)
            if n < 4 or (n % 2) != 0:
                return v
            half = n // 2
            # Heuristic 1: first half ~ second half (duplicated block)
            half_err = sum(abs(v[i] - v[i + half]) for i in range(half)) / float(half)
            if half_err <= 3.0:
                return _stretch_to_bars(v[:half], n)
            # Heuristic 2: adjacent pairs ~ equal (a,a,b,b,...)
            pair_err = sum(abs(v[i] - v[i + 1]) for i in range(0, n, 2)) / float(half)
            if pair_err <= 3.0:
                return _stretch_to_bars(v[::2], n)
            return v

        while True:
            try:
                chunk = self.proc.stdout.read(512)
            except Exception:
                break
            if not chunk:
                break
            buf.extend(chunk)
            # Read can return arbitrary chunk sizes from pipe; consume full frames.
            while len(buf) >= frame_size:
                frame = bytes(buf[:frame_size])
                del buf[:frame_size]
                # Candidate decodes
                l_inter = [frame[i * 2] for i in range(self.bars)]
                r_inter = [frame[i * 2 + 1] for i in range(self.bars)]
                l_block = [frame[i] for i in range(self.bars)]
                r_block = [frame[self.bars + i] for i in range(self.bars)]

                layout = CAVA_STEREO_LAYOUT
                if layout == "auto":
                    if self._locked_layout is None:
                        # Pick the layout that yields better channel separation.
                        score_inter = sum(abs(l_inter[i] - r_inter[i]) for i in range(self.bars))
                        score_block = sum(abs(l_block[i] - r_block[i]) for i in range(self.bars))
                        if score_block > score_inter:
                            self._auto_votes_block += 1
                        else:
                            self._auto_votes_inter += 1
                        total_votes = self._auto_votes_block + self._auto_votes_inter
                        if total_votes >= 40:
                            self._locked_layout = "block" if self._auto_votes_block > self._auto_votes_inter else "interleaved"
                    layout = self._locked_layout if self._locked_layout else "interleaved"

                if layout == "block":
                    l = l_block
                    r = r_block
                else:
                    l = l_inter
                    r = r_inter
                # Some cava/device combos can produce duplicated bin patterns
                # (e.g. L,L or repeated halves). Normalize to avoid mirrored duplicates.
                l = _dedup_if_needed(l)
                r = _dedup_if_needed(r)
                with self.lock:
                    self.left = l
                    self.right = r
                    self.frames += 1
                    self.last_frame_ts = time.time()

    def get_fft_line(self) -> str:
        with self.lock:
            l = self.left[:]
            r = self.right[:]
        left_csv = ",".join(str(v) for v in l)
        right_csv = ",".join(str(v) for v in r)
        return f"FFT|{left_csv}|{right_csv}"


class ProxyFftBridge:
    def __init__(self, bars: int):
        self.bars = bars
        self.last_state = "stop"
        self.last_volume = 35
        self.last_state_fetch = 0.0
        self.last_state_ok = 0.0
        self.has_track = False
        self.rng = random.Random(7439)
        self.band_shape_l = [self.rng.uniform(0.35, 1.00) for _ in range(bars)]
        self.band_shape_r = [self.rng.uniform(0.35, 1.00) for _ in range(bars)]
        self.band_contour = [1.16, 1.11, 1.04, 0.98, 0.93, 0.90, 0.90, 0.93, 0.98, 1.03, 1.08, 1.12]
        self.prev_l = [0.0] * bars
        self.prev_r = [0.0] * bars

    def _refresh_state(self):
        now = time.time()
        if now - self.last_state_fetch < 0.35:
            return
        self.last_state_fetch = now
        line = get_state_line()
        # line format: STAT|state|title|artist|file
        if not line.startswith("STAT|"):
            return
        parts = line.split("|", 4)
        if len(parts) != 5:
            return
        self.last_state = parts[1].strip().lower()
        self.has_track = any(p.strip() and p.strip() != "-" for p in parts[2:])
        self.last_state_ok = now
        try:
            vol_proc = run_cmd(["mpc", "volume"])
            text = vol_proc.stdout.strip().lower()
            if "volume:" in text and "%" in text:
                val = text.split("volume:", 1)[1].split("%", 1)[0].strip()
                self.last_volume = int(val)
        except Exception:
            pass

    def get_fft_line(self) -> str:
        self._refresh_state()
        is_playing = self.last_state in ("play", "playing")
        state_fresh = (time.time() - self.last_state_ok) < 2.0
        active = is_playing and self.has_track and state_fresh
        vol = max(0, min(100, int(self.last_volume)))
        amp = min(255, int(10 + vol * 2.2))

        left = [0] * self.bars
        right = [0] * self.bars
        frame_gain = self.rng.uniform(0.90, 1.14)
        for i in range(self.bars):
            if active:
                shape_l = self.band_shape_l[i] * self.band_contour[i] * frame_gain
                shape_r = self.band_shape_r[i] * self.band_contour[i] * frame_gain
                target_l = amp * shape_l + self.rng.uniform(-0.20, 0.20) * amp
                target_r = amp * shape_r + self.rng.uniform(-0.20, 0.20) * amp
                target_l = max(0.0, min(255.0, target_l))
                target_r = max(0.0, min(255.0, target_r))
                self.prev_l[i] = self.prev_l[i] * 0.62 + target_l * 0.38
                self.prev_r[i] = self.prev_r[i] * 0.62 + target_r * 0.38
            else:
                self.prev_l[i] *= 0.35
                self.prev_r[i] *= 0.35
                if self.prev_l[i] < 1.0:
                    self.prev_l[i] = 0.0
                if self.prev_r[i] < 1.0:
                    self.prev_r[i] = 0.0

            left[i] = int(max(0.0, min(255.0, self.prev_l[i])))
            right[i] = int(max(0.0, min(255.0, self.prev_r[i])))

        left_csv = ",".join(str(v) for v in left)
        right_csv = ",".join(str(v) for v in right)
        return f"FFT|{left_csv}|{right_csv}"


def main():
    ser = None
    print("moode bridge starting on", SERIAL_PORT)
    last_push = 0.0
    last_fft_push = 0.0
    last_state_tx = 0.0
    last_state_refresh = 0.0
    last_sent_state_line = ""
    state_line_cache = "STAT|unknown|-|-|-"
    fft_provider = None
    last_cava_restart_ts = 0.0
    cava_start_grace_until = 0.0
    burst_until = 0.0
    print(f"FFT enabled: {ENABLE_FFT}")
    if ENABLE_FFT:
        if USE_PROXY_FFT:
            fft_provider = ProxyFftBridge(FFT_BARS)
            print("proxy FFT enabled")
        else:
            cava = CavaBridge(FFT_BARS)
            started = cava.start()
            print(f"cava started: {started}")
            if started:
                fft_provider = cava
                cava_start_grace_until = time.time() + 12.0
            else:
                fft_provider = None
                print("cava unavailable, FFT disabled (no proxy fallback)")

    def ensure_serial():
        nonlocal ser
        if ser is not None and ser.is_open:
            return True
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD, timeout=SERIAL_READ_TIMEOUT_S)
            print("moode bridge connected on", SERIAL_PORT)
            return True
        except serial.SerialException as e:
            print("serial open failed:", e)
            time.sleep(1.0)
            return False

    while True:
        if not ensure_serial():
            continue
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
            # Keep FFT high priority and regular to avoid visible micro-freezes.
            if fft_provider is not None and now - last_fft_push >= FFT_PUSH_INTERVAL_S:
                ser.write((fft_provider.get_fft_line() + "\n").encode("utf-8"))
                last_fft_push = now

            # Refresh expensive MPD/Moode state less frequently; push from cache.
            refresh_period = 0.35 if now < burst_until else STATE_REFRESH_INTERVAL_S
            if now - last_state_refresh >= refresh_period:
                # Prefer full metadata path for reliability (artist/title correctness).
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
            # Self-heal: if cava stream stalls, recreate it so bargraph recovers.
            if isinstance(fft_provider, CavaBridge):
                stale = (
                    fft_provider.last_frame_ts > 0
                    and now > cava_start_grace_until
                    and (now - fft_provider.last_frame_ts) > 12.0
                )
                proc_dead = (fft_provider.proc is None) or (fft_provider.proc.poll() is not None)
                # Throttle restarts to avoid fast restart loops.
                if (stale or proc_dead) and (now - last_cava_restart_ts) > 10.0:
                    fft_provider.stop()
                    restarted = fft_provider.start()
                    if restarted:
                        last_cava_restart_ts = now
                        cava_start_grace_until = now + 12.0
                        print("cava restarted after stall/process exit")
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
