#!/usr/bin/env python3
import json
import os
import random
import subprocess
import tempfile
import threading
import time
from urllib import request

import serial

SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/serial0")
BAUD = 115200
VOLUMIO_BASE = "http://127.0.0.1:3000"
FFT_BARS = 12
# FFT enabled by default; disable only with ENABLE_FFT=0
ENABLE_FFT = os.getenv("ENABLE_FFT", "1") == "1"
FFT_PUSH_INTERVAL_S = 0.16  # ~6 FPS on serial, safer for Volumio
CAVA_FRAMERATE = 25         # lower analyzer CPU usage
CAVA_ALSA_SOURCE = os.getenv("CAVA_ALSA_SOURCE", "plughw:Loopback,1,0")
# Real FFT recommended mode: fifo (from MPD), without touching DAC output path.
CAVA_INPUT_MODE = os.getenv("CAVA_INPUT_MODE", "fifo")  # fifo | alsa
CAVA_FIFO_PATH = os.getenv("CAVA_FIFO_PATH", "/tmp/mpd.fifo")
USE_PROXY_FFT = os.getenv("USE_PROXY_FFT", "0") == "1"


def http_get_json(path: str):
    with request.urlopen(VOLUMIO_BASE + path, timeout=2.0) as resp:
        data = resp.read().decode("utf-8", errors="ignore")
        return json.loads(data)


def http_post(path: str):
    req = request.Request(VOLUMIO_BASE + path, method="POST")
    with request.urlopen(req, timeout=2.0):
        return True


def http_get(path: str):
    with request.urlopen(VOLUMIO_BASE + path, timeout=2.0):
        return True


def get_state_line():
    try:
        s = http_get_json("/api/v1/getState")
    except Exception:
        return "STAT|offline|-|-|-"

    status = str(s.get("status", "unknown"))
    title = str(s.get("title", "-")).replace("|", "/")
    artist = str(s.get("artist", "-")).replace("|", "/")
    uri = str(s.get("uri", "-")).replace("|", "/")
    return f"STAT|{status}|{title}|{artist}|{uri}"


def run_command(cmd: str):
    def cmd_call(path: str):
        # Volumio builds can differ: some accept POST, others react only to GET.
        try:
            return http_post(path)
        except Exception:
            return http_get(path)

    cmd = cmd.strip().upper()
    if cmd == "PREV":
        cmd_call("/api/v1/commands/?cmd=prev")
    elif cmd == "NEXT":
        cmd_call("/api/v1/commands/?cmd=next")
    elif cmd in ("PLAYPAUSE", "TOGGLE"):
        cmd_call("/api/v1/commands/?cmd=toggle")
    elif cmd == "PLAY":
        cmd_call("/api/v1/commands/?cmd=play")
    elif cmd == "PAUSE":
        cmd_call("/api/v1/commands/?cmd=pause")
    elif cmd == "STOP":
        cmd_call("/api/v1/commands/?cmd=stop")
    elif cmd.startswith("VOLUME="):
        vol = int(cmd.split("=", 1)[1])
        vol = max(0, min(100, vol))
        cmd_call("/api/v1/commands/?cmd=volume&volume=" + str(vol))


class CavaBridge:
    def __init__(self, bars: int):
        self.bars = bars
        self.left = [0] * bars
        self.right = [0] * bars
        self.lock = threading.Lock()
        self.proc = None
        self.cfg_path = None
        self.thread = None

    def start(self):
        if CAVA_INPUT_MODE == "fifo":
            if not os.path.exists(CAVA_FIFO_PATH):
                os.mkfifo(CAVA_FIFO_PATH)
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
                ["cava", "-p", self.cfg_path],
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

    def _reader_loop(self):
        if not self.proc or not self.proc.stdout:
            return
        frame_size = self.bars * 2
        while True:
            try:
                chunk = self.proc.stdout.read(frame_size)
            except Exception:
                break
            if not chunk or len(chunk) < frame_size:
                break
            l = [chunk[i * 2] for i in range(self.bars)]
            r = [chunk[i * 2 + 1] for i in range(self.bars)]
            with self.lock:
                self.left = l
                self.right = r

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
        self.last_progress_ok = 0.0
        self.has_track = False
        self.last_seek = -1
        self.rng = random.Random(7439)
        # Wider per-band variation so the graph is less flat.
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
        try:
            s = http_get_json("/api/v1/getState")
            self.last_state = str(s.get("status", "stop")).lower()
            self.last_volume = int(s.get("volume", 35))
            title = str(s.get("title", "")).strip()
            artist = str(s.get("artist", "")).strip()
            uri = str(s.get("uri", "")).strip()
            self.has_track = any(v and v != "-" for v in (title, artist, uri))
            seek = int(s.get("seek", -1))
            if seek >= 0:
                if self.last_seek >= 0 and seek > self.last_seek:
                    self.last_progress_ok = now
                self.last_seek = seek
            if self.last_state in ("play", "playing") and self.last_progress_ok == 0.0:
                # Allow brief grace on startup before first seek increment arrives.
                self.last_progress_ok = now
            self.last_state_ok = now
        except Exception:
            pass

    def get_fft_line(self) -> str:
        self._refresh_state()
        is_playing = self.last_state in ("play", "playing")
        state_fresh = (time.time() - self.last_state_ok) < 2.0
        progress_fresh = (time.time() - self.last_progress_ok) < 1.8
        active = is_playing and self.has_track and state_fresh and progress_fresh
        vol = max(0, min(100, int(self.last_volume)))
        amp = int(10 + vol * 2.2)
        if amp > 255:
            amp = 255

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
                # Smoothing keeps movement natural without directional scrolling.
                self.prev_l[i] = self.prev_l[i] * 0.62 + target_l * 0.38
                self.prev_r[i] = self.prev_r[i] * 0.62 + target_r * 0.38
            else:
                # No song / stale state: drop quickly to zero, no fake activity.
                if not self.has_track or not state_fresh or self.last_state in ("stop", "stopped"):
                    self.prev_l[i] *= 0.35
                    self.prev_r[i] *= 0.35
                else:
                    # Pause: softer decay.
                    self.prev_l[i] *= 0.55
                    self.prev_r[i] *= 0.55
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
    print("volumio bridge starting on", SERIAL_PORT)
    last_push = 0.0
    last_fft_push = 0.0
    fft_provider = None
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
            else:
                fft_provider = ProxyFftBridge(FFT_BARS)
                print("cava unavailable, fallback to proxy FFT")

    def ensure_serial():
        nonlocal ser
        if ser is not None and ser.is_open:
            return True
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
            print("volumio bridge connected on", SERIAL_PORT)
            return True
        except serial.SerialException as e:
            print("serial open failed:", e)
            time.sleep(1.0)
            return False

    while True:
        if not ensure_serial():
            continue
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("CMD|"):
                try:
                    run_command(line.split("|", 1)[1])
                except Exception:
                    pass
                ser.write((get_state_line() + "\n").encode("utf-8"))
            elif line == "GET":
                ser.write((get_state_line() + "\n").encode("utf-8"))

            # periodic push (helps UI refresh even without GET polling)
            now = time.time()
            if now - last_push >= 2.0:
                ser.write((get_state_line() + "\n").encode("utf-8"))
                last_push = now
            if fft_provider is not None and now - last_fft_push >= FFT_PUSH_INTERVAL_S:
                ser.write((fft_provider.get_fft_line() + "\n").encode("utf-8"))
                last_fft_push = now
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
