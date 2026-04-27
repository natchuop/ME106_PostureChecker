"""
laptopPostureCheck.py — Posture monitor for Windows + Raspberry Pi OS (Linux)

Same codebase runs on both:
  • Windows laptop (dev/testing) — integrated or USB webcam (via V4L2/MSMF).
  • Raspberry Pi Zero 2 W (deployment) — CSI Pi Camera via rpicam-vid, or
    USB webcam on /dev/video0.
Both platforms host the annotated live stream over HTTP (multipart MJPEG).
MediaPipe pose runs on a background thread so the stream isn't gated by it.

INSTALL (Windows):
    pip install mediapipe opencv-python-headless pyserial flask

INSTALL (Pi OS Bookworm/Trixie, 64-bit, Python 3.11/3.12 venv):
    sudo apt install -y rpicam-apps libopenblas-dev libgl1
    pip install mediapipe opencv-python-headless pyserial flask

ACCESS STREAM (point browser at the host running this script):
    Pi:      http://<pi-ip>:5000/
    Windows: http://127.0.0.1:8765/   (port 5000 collides with AirPlay etc.)
Env overrides: STREAM_PORT, CAMERA_INDEX, SERIAL_PORT.
"""

import cv2
import numpy as np
import serial
import time
import os
import sys
import socket
import platform
import threading
import shutil
import subprocess

# ─────────────────────────────────────────────
# PLATFORM + MEDIAPIPE
# ─────────────────────────────────────────────

IS_WINDOWS = platform.system() == "Windows"
IS_LINUX = platform.system() == "Linux"

# TFLite / XNNPACK threads = 2 on the Pi: try to parallelize inference across
# two cores while keeping two cores cold for thermal headroom.
if IS_LINUX:
    os.environ.setdefault("OMP_NUM_THREADS", "2")
    os.environ.setdefault("TFLITE_NUM_THREADS", "2")
    os.environ.setdefault("MEDIAPIPE_TFLITE_NUM_THREADS", "2")
    os.environ.setdefault("XNNPACK_NUM_THREADS", "2")
    os.environ.setdefault("GLOG_minloglevel", "2")

try:
    import mediapipe as mp
    mp_pose = mp.solutions.pose
    print("[INFO] MediaPipe loaded successfully")
except Exception as e:
    print(f"[ERROR] MediaPipe failed to load: {e}")
    print("Run: pip install mediapipe  (Pi OS: pip install --break-system-packages mediapipe)")
    sys.exit(1)

# ─────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────

SERIAL_PORT = os.environ.get(
    "SERIAL_PORT", "COM10" if IS_WINDOWS else "/dev/ttyACM0"
)
BAUD_RATE = 9600

FRAME_WIDTH = 256
FRAME_HEIGHT = 192
CENTER_DEADZONE = 5
# Drop "person lost" only after this many misses (brief occlusions recover).
PERSON_LOST_STREAK = 4
# Confirm bad posture only after holding it for this long (seconds).
BAD_HOLD_SECONDS = 3.0
# Windows port 5000 often collides with AirPlay / other services; Pi is free to use 5000.
STREAM_PORT = int(os.environ.get("STREAM_PORT", "8765" if IS_WINDOWS else "5000"))
STREAM_FPS = 4           # target frames/sec pushed to browser
STREAM_JPEG_QUALITY = 35 # 0-100; lower = less bandwidth, more artifacting

# Webcam index (USB / integrated only — CSI Pi Camera auto-detected separately).
CAMERA_INDEX = int(os.environ.get("CAMERA_INDEX", "1" if IS_WINDOWS else "0"))

# Posture thresholds. Normalized landmarks (0..1, y grows downward) unless noted _DEG.
# Webcam is mostly frontal; 2D proxies stand in for true sagittal measurements.
# Side/torso cues (spine_angle + torso_lean only fire when hips are visible).
SPINE_ANGLE_MIN_DEG = 156.0            # nose-mid_shoulder-mid_hip; primary spine/torso check
SHOULDER_HIP_HORIZONTAL_MAX = 0.052    # mid_shoulder.x vs mid_hip.x; forward-lean proxy
# Head / neck cues (work without hips)
FOREHEAD_SHOULDER_CLEARANCE_MIN = 0.118 # face top too close to shoulders (FHP proxy)
TRAP_NECK_CLEARANCE_MIN = 0.058        # ears too close vertically to shoulders (raised shoulders)
CHIN_BELOW_EAR_Y = 0.045               # chin dropped below ear line
HEAD_OFF_CENTER_X_MAX = 0.068          # |nose.x − mid_shoulder.x|; head shifted off center
# Symmetry cues (angle-based, distance independent)
SHOULDER_SLOPE_MAX_DEG = 5.5           # shoulder line tilt vs horizontal
EAR_SLOPE_MAX_DEG = 6.5                # ear line tilt vs horizontal (head tilt)
# Visibility gates
HIP_MIN_VISIBILITY = 0.35
NOSE_MIN_VISIBILITY = 0.2
EAR_MIN_VISIBILITY = 0.22
FOREHEAD_VIS_MIN = 0.18

# ─────────────────────────────────────────────
# SERIAL SETUP
# ─────────────────────────────────────────────

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"[SERIAL] Connected to Pico on {SERIAL_PORT}")
    # Match testPicoComms.py startup handshake:
    # 1) stop any running script in REPL, 2) soft reboot so main.py runs fresh.
    ser.write(b"\x03")  # Ctrl+C
    time.sleep(0.5)
    ser.write(b"\x04")  # Ctrl+D
    time.sleep(3.0)     # allow main.py + hardware init to complete
except Exception as e:
    print(f"[SERIAL] No Pico on {SERIAL_PORT} ({e}). Continuing without motor/fire commands.")

# ─────────────────────────────────────────────
# CAMERA SETUP
# ─────────────────────────────────────────────

def _frame_brightness(frame):
    """Mean pixel level; ~0 = black (wrong/disabled device), higher = real picture."""
    if frame is None or frame.size == 0:
        return 0.0
    return float(np.mean(frame))


class PiCamMJPEG:
    """
    cv2.VideoCapture-compatible wrapper around `rpicam-vid` for the Pi CSI
    camera (libcamera stack on Pi OS Bookworm/Trixie). Spawns rpicam-vid as a
    subprocess streaming MJPEG to stdout; a background reader thread drains
    the pipe continuously and keeps only the most recent decoded BGR frame,
    so read() always returns a near-real-time frame even when the main loop
    (e.g. MediaPipe) runs slower than the camera's output rate.

    Only the subset of the VideoCapture API this script uses is implemented:
    read(), release(), isOpened().
    """

    _SOI = b"\xff\xd8"
    _EOI = b"\xff\xd9"

    def __init__(self, width, height, fps=15):
        bin_name = "rpicam-vid" if shutil.which("rpicam-vid") else (
            "libcamera-vid" if shutil.which("libcamera-vid") else None
        )
        if bin_name is None:
            raise RuntimeError("rpicam-vid / libcamera-vid not installed")
        cmd = [
            bin_name, "-t", "0", "--codec", "mjpeg", "-o", "-",
            "--width", str(width), "--height", str(height),
            "--framerate", str(fps), "--nopreview", "-n",
            "--flush",
        ]
        self._proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0
        )
        self._latest = None
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._frame_id = 0
        self._last_seen_id = 0
        self._opened = True
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

    def _reader_loop(self):
        """Continuously parse MJPEG stream and overwrite self._latest."""
        buf = bytearray()
        read = self._proc.stdout.read
        while self._opened:
            chunk = read(65536)
            if not chunk:
                if self._proc.poll() is not None:
                    break
                continue
            buf.extend(chunk)

            while True:
                start = buf.find(self._SOI)
                if start < 0:
                    buf.clear()
                    break
                end = buf.find(self._EOI, start + 2)
                if end < 0:
                    if start > 0:
                        del buf[:start]
                    break
                jpeg = bytes(buf[start:end + 2])
                del buf[:end + 2]
                arr = np.frombuffer(jpeg, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                with self._cond:
                    self._latest = frame
                    self._frame_id += 1
                    self._cond.notify_all()

    def isOpened(self):
        return self._opened and self._proc.poll() is None

    def set(self, *_args, **_kwargs):
        return True

    def read(self):
        """
        Return (ret, frame_bgr) with the latest available frame.
        Waits up to 2s for a *new* frame (one we haven't returned before) to
        keep timestamps monotonic; if none arrives, returns the cached one.
        """
        with self._cond:
            if not self._cond.wait_for(
                lambda: self._frame_id > self._last_seen_id or self._latest is not None,
                timeout=2.0,
            ):
                return False, None
            self._last_seen_id = self._frame_id
            return (self._latest is not None), self._latest

    def release(self):
        self._opened = False
        try:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        except Exception:
            pass


def _has_csi_camera():
    """Best-effort check: is a libcamera-visible camera attached?"""
    if shutil.which("rpicam-vid") is None and shutil.which("libcamera-vid") is None:
        print("[CAMERA] rpicam-vid / libcamera-vid not found — install with: sudo apt install rpicam-apps")
        return False
    bin_name = "rpicam-hello" if shutil.which("rpicam-hello") else (
        "libcamera-hello" if shutil.which("libcamera-hello") else None
    )
    if bin_name is None:
        return True
    try:
        res = subprocess.run(
            [bin_name, "--list-cameras"],
            capture_output=True, text=True, timeout=5,
        )
        combined = (res.stdout + res.stderr).lower()
        if "no cameras available" in combined:
            return False
        if res.returncode != 0 and "available cameras" not in combined:
            return False
        # Pull just the "0 : sensor_name ..." line for a concise summary.
        summary = next(
            (ln.strip() for ln in res.stdout.splitlines() if ln.strip().startswith("0 :")),
            "camera present",
        )
        print(f"[CAMERA] libcamera: {summary}")
        return True
    except Exception as e:
        print(f"[CAMERA] {bin_name} check failed: {e}")
        return False


def open_camera():
    """
    Find a capture device that actually delivers non-black frames.
    On Windows, try Media Foundation before DirectShow — DSHOW often opens a
    phantom/virtual device that reads OK but is all black.
    On Linux, if a CSI Pi Camera is detected, stream via rpicam-vid (MJPEG).
    """
    if IS_LINUX and _has_csi_camera():
        try:
            print("[CAMERA] CSI Pi Camera detected — using rpicam-vid (MJPEG)")
            pi = PiCamMJPEG(FRAME_WIDTH, FRAME_HEIGHT, fps=6)
            # rpicam-vid cold start can take 3-6s (libcamera init + AGC/AEC
            # convergence). Be patient: poll up to 15s for a usable frame, with
            # a progress dot every second so the user sees it working.
            warmup_deadline = time.time() + 15.0
            best = 0.0
            last_frame = None
            last_tick = time.time()
            print("[CAMERA] Warming up CSI camera", end="", flush=True)
            while time.time() < warmup_deadline:
                ret, frm = pi.read()
                if ret and frm is not None:
                    last_frame = frm
                    b = _frame_brightness(frm)
                    if b > best:
                        best = b
                    if b >= 2.0:
                        print(f"\n[CAMERA] Using CSI Pi Camera — brightness mean ≈ {b:.1f}")
                        return pi
                if time.time() - last_tick > 1.0:
                    print(".", end="", flush=True)
                    last_tick = time.time()
                time.sleep(0.05)
            if last_frame is not None:
                print(f"\n[CAMERA] CSI camera warmed up but dim (best mean ≈ {best:.1f}); using it anyway")
                return pi
            print("\n[CAMERA] CSI camera opened but delivered no frames; falling back to V4L2")
            pi.release()
        except Exception as e:
            print(f"[CAMERA] CSI Pi Camera init failed: {e}; falling back to V4L2")

    if CAMERA_INDEX is not None:
        try:
            indices = [int(CAMERA_INDEX)]
        except ValueError:
            indices = list(range(6))
    else:
        indices = list(range(6))

    if IS_WINDOWS:
        # MSMF first: often the real integrated webcam; DSHOW second for compatibility
        backends = [
            (cv2.CAP_MSMF, "Media Foundation"),
            (cv2.CAP_DSHOW, "DirectShow"),
        ]
    elif IS_LINUX:
        # V4L2 is the right backend for USB webcams on Raspberry Pi OS / Linux.
        backends = [
            (cv2.CAP_V4L2, "V4L2"),
            (None, "default"),
        ]
    else:
        backends = [(None, "default")]

    # Reject feeds that look like a dead/blank device (not the same as a dark room)
    min_brightness = 6.0

    for idx in indices:
        for api, label in backends:
            cap_try = cv2.VideoCapture(idx, api) if api is not None else cv2.VideoCapture(idx)
            if not cap_try.isOpened():
                cap_try.release()
                continue
            try:
                cap_try.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            cap_try.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            cap_try.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
            cap_try.set(cv2.CAP_PROP_FPS, 15)

            for _ in range(25):
                cap_try.read()

            best = 0.0
            ok = False
            for _ in range(45):
                ret, frm = cap_try.read()
                if not ret or frm is None:
                    time.sleep(0.02)
                    continue
                b = _frame_brightness(frm)
                if b > best:
                    best = b
                if b >= min_brightness:
                    ok = True
                    print(
                        f"[CAMERA] Using index {idx} ({label}) — "
                        f"brightness mean ≈ {b:.1f} (0–255)"
                    )
                    return cap_try
                time.sleep(0.02)

            if best >= 2.0:
                print(
                    f"[CAMERA] Using index {idx} ({label}) — very dark feed "
                    f"(mean ≈ {best:.1f}); add light or edit CAMERA_INDEX in this file"
                )
                return cap_try

            print(f"[CAMERA] Skipping index {idx} ({label}) — looks blank (best mean ≈ {best:.1f})")
            cap_try.release()

    return None


cap = open_camera()
if cap is None:
    print("[CAMERA] No usable camera — devices were missing, all-black, or unreadable.")
    if IS_WINDOWS:
        print("  • Settings → Privacy & security → Camera → allow desktop apps")
        print("  • Quit other apps using the camera; set env CAMERA_INDEX=0/1/2")
        print("  • Integrated vs USB webcam: try another index until brightness looks right in the console")
    elif IS_LINUX:
        print("  • Check that a camera exists: ls /dev/video*")
        print("  • Set env CAMERA_INDEX to match (e.g. /dev/video0 → CAMERA_INDEX=0)")
        print("  • USB webcams work out of the box via V4L2; the CSI Pi Camera module needs")
        print("    'sudo raspi-config' → legacy camera, or a libcamera/picamera2 adaptation")
    sys.exit(1)

# ─────────────────────────────────────────────
# MEDIAPIPE POSE SETUP
# ─────────────────────────────────────────────

pose = mp_pose.Pose(
    model_complexity=0,
    smooth_landmarks=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)

# ─────────────────────────────────────────────
# STREAMING (unified Flask multipart MJPEG, works on Windows + Pi)
# ─────────────────────────────────────────────

latest_frame = None


def _discover_lan_ip():
    """Find an outbound-facing IP without actually sending. Robust to /etc/hosts loopback
    aliases (Raspberry Pi OS maps the hostname to 127.0.1.1 by default)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def start_stream():
    """HTTP server that serves an MJPEG stream with the posture overlay baked in.
    Single long-lived connection per viewer (multipart/x-mixed-replace) — efficient
    enough for a Pi Zero 2 W and for a laptop alike."""
    from flask import Flask, Response

    app = Flask(__name__)
    # Silence the noisy per-request access log (Pi Zero benefits from less I/O too).
    import logging
    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    frame_period = 1.0 / max(1, STREAM_FPS)
    jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, STREAM_JPEG_QUALITY]
    boundary = b"--frame"

    def mjpeg_generator():
        placeholder = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
        while True:
            frame = latest_frame if latest_frame is not None else placeholder
            ok, buf = cv2.imencode(".jpg", frame, jpeg_params)
            if ok and buf is not None:
                payload = buf.tobytes()
                yield (
                    boundary + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(payload)).encode() + b"\r\n\r\n"
                    + payload + b"\r\n"
                )
            time.sleep(frame_period)

    @app.route("/stream")
    def stream():
        return Response(
            mjpeg_generator(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-cache, no-store, must-revalidate", "Pragma": "no-cache"},
        )

    @app.route("/")
    def index():
        return """
        <html>
        <head>
            <title>Posture Cam</title>
            <style>
                body { background:#111; color:#eee; font-family:sans-serif; text-align:center; padding:20px; }
                img { border:2px solid #444; border-radius:8px; max-width:100%; height:auto; image-rendering: pixelated; }
                .meta { color:#888; font-size:12px; margin-top:8px; }
            </style>
        </head>
        <body>
            <h2>Posture Cam — Live Feed</h2>
            <img src="/stream" alt="camera feed" />
            <div class="meta">MJPEG stream at <code>/stream</code></div>
        </body>
        </html>"""

    lan_ip = _discover_lan_ip()
    print(f"[STREAM] Live feed at http://{lan_ip}:{STREAM_PORT}/")
    if IS_WINDOWS:
        print(f"[STREAM] Local:          http://127.0.0.1:{STREAM_PORT}/")

    thread = threading.Thread(
        target=lambda: app.run(
            host="0.0.0.0", port=STREAM_PORT, threaded=True,
            use_reloader=False, debug=False,
        ),
        daemon=True,
    )
    thread.start()
    time.sleep(0.2)  # let the socket bind before the main loop starts producing frames


start_stream()

# ─────────────────────────────────────────────
# SERIAL HELPERS
# ─────────────────────────────────────────────

def send_command(cmd):
    if ser:
        try:
            ser.write(f"{cmd}\n".encode())
        except Exception as e:
            print(f"[SERIAL] Error: {e}")

def _serial_reader_thread():
    """Forward any lines the Pico prints over USB serial to the laptop terminal."""
    while True:
        if ser:
            try:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    print(f"[PICO] {line}")
            except Exception:
                pass

if ser:
    threading.Thread(target=_serial_reader_thread, daemon=True).start()
    send_command("off")   # ensure Pico starts in off state
    print("[CONTROL] Pico defaulting to OFF — type 'on' to activate")

def rotate_left(error):
    send_command(f"move:{error}")

def rotate_right(error):
    send_command(f"move:{error}")

def stop_motor():
    send_command("stop")

def fire():
    print("[FIRE] Firing — cooldown ~8 seconds")
    send_command("fire")
    threading.Timer(8.0, lambda: print("[FIRE] Cooldown complete — ready to fire again")).start()

# ─────────────────────────────────────────────
# POSTURE (sagittal spine curvature + head / neck)
# ─────────────────────────────────────────────

plm = mp_pose.PoseLandmark


def spine_angle_deg(lm):
    """
    Angle at mid-shoulder between vectors to nose and to mid_hip (sagittal spine bend).
    ~180° = straighter; lower = more forward curvature / slouch.
    """
    nose = lm[plm.NOSE]
    ls, rs = lm[plm.LEFT_SHOULDER], lm[plm.RIGHT_SHOULDER]
    lh, rh = lm[plm.LEFT_HIP], lm[plm.RIGHT_HIP]
    mid_sx = (ls.x + rs.x) * 0.5
    mid_sy = (ls.y + rs.y) * 0.5
    mid_hx = (lh.x + rh.x) * 0.5
    mid_hy = (lh.y + rh.y) * 0.5
    v1 = np.array([nose.x - mid_sx, nose.y - mid_sy], dtype=np.float64)
    v2 = np.array([mid_hx - mid_sx, mid_hy - mid_sy], dtype=np.float64)
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return None
    c = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


def _face_top_y(lm):
    """
    Smallest y among visible nose + eye points = highest part of face in the image
    (proxy for forehead / upper face). Dips toward shoulders when slouching.
    """
    ys = []
    for i in range(7):
        p = lm[i]
        if getattr(p, "visibility", 1.0) >= FOREHEAD_VIS_MIN:
            ys.append(p.y)
    if ys:
        return min(ys)
    n = lm[plm.NOSE]
    if getattr(n, "visibility", 1.0) >= NOSE_MIN_VISIBILITY:
        return n.y
    return None


def _line_slope_deg(ax, ay, bx, by):
    """Tilt of segment A-B vs horizontal, in degrees, always non-negative."""
    dx = abs(bx - ax)
    dy = abs(by - ay)
    if dx < 1e-6:
        return 90.0
    return float(np.degrees(np.arctan2(dy, dx)))


def evaluate_posture(landmarks):
    """
    Returns (is_bad, reasons). Two buckets:
      Side/torso cues (any one alone => bad):
        - spine_angle: nose-mid_shoulder-mid_hip angle < SPINE_ANGLE_MIN_DEG (hips only)
        - forehead_dip: face top too close to shoulders (FHP, no hips needed)
        - trap_neck:    ears too close vertically to shoulders (raised/hunched)
        - chin_down:    chin below ear line
        - torso_lean:   mid_shoulder horizontally offset from mid_hip (hips only)
      Front cues (shoulder_slope alone OR 2+ signals => bad):
        - shoulder_slope (angle), ear_slope (angle), head_offcenter
    """
    lm = landmarks.landmark
    nose = lm[plm.NOSE]
    ls, rs = lm[plm.LEFT_SHOULDER], lm[plm.RIGHT_SHOULDER]
    lh, rh = lm[plm.LEFT_HIP], lm[plm.RIGHT_HIP]
    lea, rea = lm[plm.LEFT_EAR], lm[plm.RIGHT_EAR]

    mid_sx = (ls.x + rs.x) * 0.5
    mid_sy = (ls.y + rs.y) * 0.5
    mid_hx = (lh.x + rh.x) * 0.5

    hip_vis = min(getattr(lh, "visibility", 1.0), getattr(rh, "visibility", 1.0))
    hips_visible = hip_vis >= HIP_MIN_VISIBILITY
    nose_vis = getattr(nose, "visibility", 1.0)
    ears_ok = (
        getattr(lea, "visibility", 1.0) >= EAR_MIN_VISIBILITY
        and getattr(rea, "visibility", 1.0) >= EAR_MIN_VISIBILITY
    )

    reasons = set()

    # --- Side/torso cues ---
    if hips_visible and nose_vis >= NOSE_MIN_VISIBILITY:
        ang = spine_angle_deg(lm)
        if ang is not None and ang < SPINE_ANGLE_MIN_DEG:
            reasons.add("spine_angle")

    face_top_y = _face_top_y(lm)
    if face_top_y is not None and (mid_sy - face_top_y) < FOREHEAD_SHOULDER_CLEARANCE_MIN:
        reasons.add("forehead_dip")

    if ears_ok:
        ear_mid_y = (lea.y + rea.y) * 0.5
        if (mid_sy - ear_mid_y) < TRAP_NECK_CLEARANCE_MIN:
            reasons.add("trap_neck")
        if nose_vis >= NOSE_MIN_VISIBILITY and (nose.y - ear_mid_y) > CHIN_BELOW_EAR_Y:
            reasons.add("chin_down")

    if hips_visible and abs(mid_sx - mid_hx) > SHOULDER_HIP_HORIZONTAL_MAX:
        reasons.add("torso_lean")

    strong_side = {"spine_angle", "forehead_dip", "trap_neck", "chin_down", "torso_lean"}
    if reasons & strong_side:
        return True, reasons

    # --- Front cues (angle-based; distance independent) ---
    shoulder_slope = _line_slope_deg(ls.x, ls.y, rs.x, rs.y)
    ear_slope = _line_slope_deg(lea.x, lea.y, rea.x, rea.y) if ears_ok else None
    shoulder_bad = shoulder_slope > SHOULDER_SLOPE_MAX_DEG
    ear_bad = ear_slope is not None and ear_slope > EAR_SLOPE_MAX_DEG
    head_offcenter = nose_vis >= NOSE_MIN_VISIBILITY and abs(nose.x - mid_sx) > HEAD_OFF_CENTER_X_MAX

    if shoulder_bad:
        reasons.add("shoulder_slope")
    if ear_bad:
        reasons.add("ear_slope")
    if head_offcenter:
        reasons.add("head_offcenter")

    if shoulder_bad:
        return True, reasons
    if sum([shoulder_bad, ear_bad, head_offcenter]) >= 2:
        return True, reasons
    return False, reasons


def _lerp(a, b, t):
    return (int(a[0] * (1 - t) + b[0] * t), int(a[1] * (1 - t) + b[1] * t))


def _pt_xy(lm, idx, w, h):
    p = lm[idx]
    return int(p.x * w), int(p.y * h)


def _draw_dense_on_connections(frame, lm, w, h, connections, steps=5, color=(160, 160, 220), r=2):
    """Extra sample nodes along given segments only."""
    for a, b in connections:
        pa = _pt_xy(lm, a, w, h)
        pb = _pt_xy(lm, b, w, h)
        for k in range(1, steps + 1):
            t = k / (steps + 1)
            cv2.circle(frame, _lerp(pa, pb, t), r, color, -1)


def _draw_dense_px(frame, pa, pb, steps=6, color=(120, 220, 200), r=2):
    for k in range(1, steps + 1):
        t = k / (steps + 1)
        cv2.circle(frame, _lerp(pa, pb, t), r, color, -1)


# Upper body only: no legs. Head/neck: nose + ears drawn separately.
TORSO_CONNECTIONS = (
    (plm.LEFT_SHOULDER, plm.RIGHT_SHOULDER),
    (plm.LEFT_SHOULDER, plm.LEFT_HIP),
    (plm.RIGHT_SHOULDER, plm.RIGHT_HIP),
    (plm.LEFT_HIP, plm.RIGHT_HIP),
)
TORSO_JOINT_INDICES = (
    plm.LEFT_SHOULDER,
    plm.RIGHT_SHOULDER,
    plm.LEFT_HIP,
    plm.RIGHT_HIP,
)


def draw_posture_overlay(frame, landmarks, slouching, reasons=None):
    """Bones + neck (nose→shoulders) + spine (mid-shoulder→mid-hip); optional ear→mid-shoulder neck base."""
    if reasons is None:
        reasons = set()
    lm = landmarks.landmark
    h, w = frame.shape[:2]
    HEAD_REASONS = {"forehead_dip", "chin_down", "trap_neck", "head_offcenter", "ear_slope"}
    SPINE_REASONS = {"spine_angle", "torso_lean"}
    HEAD_PT_REASONS = {"forehead_dip", "chin_down", "head_offcenter"}

    spine_bgr = (0, 0, 255) if (slouching and (reasons & SPINE_REASONS)) else (0, 255, 0)
    neck_bgr = (0, 0, 255) if (slouching and (reasons & HEAD_REASONS)) else (0, 200, 255)
    bone_bgr = (40, 200, 255)
    node_bgr = (200, 220, 255)
    head_bgr = (0, 0, 255) if (slouching and (reasons & HEAD_PT_REASONS)) else (200, 200, 255)

    for a, b in TORSO_CONNECTIONS:
        cv2.line(frame, _pt_xy(lm, a, w, h), _pt_xy(lm, b, w, h), bone_bgr, 2)

    ls = _pt_xy(lm, plm.LEFT_SHOULDER, w, h)
    rs = _pt_xy(lm, plm.RIGHT_SHOULDER, w, h)
    lh = _pt_xy(lm, plm.LEFT_HIP, w, h)
    rh = _pt_xy(lm, plm.RIGHT_HIP, w, h)
    mid_s = ((ls[0] + rs[0]) // 2, (ls[1] + rs[1]) // 2)
    mid_h = ((lh[0] + rh[0]) // 2, (lh[1] + rh[1]) // 2)
    nose_pt = _pt_xy(lm, plm.NOSE, w, h)
    trap_bgr = (0, 0, 255) if (slouching and "trap_neck" in reasons) else ((60, 180, 255) if slouching else (200, 120, 80))

    cv2.line(frame, nose_pt, mid_s, neck_bgr, 2)
    _draw_dense_px(frame, nose_pt, mid_s, steps=5, color=(120, 200, 255), r=2)

    cv2.line(frame, mid_s, mid_h, spine_bgr, 3)
    _draw_dense_px(frame, mid_s, mid_h, steps=7, color=(100, 255, 180), r=2)

    lea = lm[plm.LEFT_EAR]
    rea = lm[plm.RIGHT_EAR]
    ear_mid = None
    if (
        getattr(lea, "visibility", 1.0) >= EAR_MIN_VISIBILITY
        and getattr(rea, "visibility", 1.0) >= EAR_MIN_VISIBILITY
    ):
        ear_mid = (
            int((lea.x + rea.x) * 0.5 * w),
            int((lea.y + rea.y) * 0.5 * h),
        )
        cv2.line(frame, ear_mid, mid_s, (80, 180, 255), 1)
        ear_bgr = (0, 0, 255) if (slouching and "ear_slope" in reasons) else head_bgr
        cv2.circle(frame, _pt_xy(lm, plm.LEFT_EAR, w, h), 4, ear_bgr, 1)
        cv2.circle(frame, _pt_xy(lm, plm.RIGHT_EAR, w, h), 4, ear_bgr, 1)
        # Trapezius region (frontal): ears–shoulders triangle + each shoulder to neck base
        cv2.line(frame, ear_mid, ls, trap_bgr, 2)
        cv2.line(frame, ear_mid, rs, trap_bgr, 2)
        _draw_dense_px(frame, ear_mid, ls, steps=5, color=(100, 160, 255), r=2)
        _draw_dense_px(frame, ear_mid, rs, steps=5, color=(100, 160, 255), r=2)
        tri = np.array([ear_mid, rs, ls], dtype=np.int32)
        cv2.polylines(frame, [tri], True, trap_bgr, 2)

    if ear_mid is None:
        neck_top = (
            int((nose_pt[0] + mid_s[0]) * 0.5),
            int((nose_pt[1] + mid_s[1]) * 0.5),
        )
        cv2.line(frame, neck_top, ls, trap_bgr, 1)
        cv2.line(frame, neck_top, rs, trap_bgr, 1)

    cv2.circle(frame, nose_pt, 6, head_bgr, 2)

    # spine_angle_deg() / _line_slope_deg() still available for evaluate_posture()
    # and any other consumers — we just no longer print them on the stream.

    _draw_dense_on_connections(frame, lm, w, h, TORSO_CONNECTIONS, steps=4)

    for idx in TORSO_JOINT_INDICES:
        p = lm[idx]
        if getattr(p, "visibility", 1.0) < 0.2:
            continue
        # Color important joints based on which check failed.
        if idx in (plm.LEFT_SHOULDER, plm.RIGHT_SHOULDER) and slouching and "shoulder_slope" in reasons:
            c = (0, 0, 255)
        elif idx in (plm.LEFT_HIP, plm.RIGHT_HIP) and slouching and "torso_lean" in reasons:
            c = (0, 0, 255)
        else:
            c = node_bgr
        cv2.circle(frame, _pt_xy(lm, idx, w, h), 5, c, 2)

# ─────────────────────────────────────────────
# AIMING
# ─────────────────────────────────────────────

def get_person_center_x(landmarks, w):
    lm = landmarks.landmark
    center_x_norm = (lm[plm.LEFT_SHOULDER].x + lm[plm.RIGHT_SHOULDER].x) * 0.5
    return int(center_x_norm * w)

fired = False

def handle_aiming(landmarks, w):
    global fired
    frame_center = w // 2
    person_x = get_person_center_x(landmarks, w)
    error = person_x - frame_center

    if abs(error) <= CENTER_DEADZONE:
        stop_motor()
        if not fired:
            print(f"[AIM] Centered (error: {error}px) — firing")
            time.sleep(0.2)
            fire()
            fired = True
    elif error < 0:
        rotate_left(error)
    else:
        rotate_right(error)

# ─────────────────────────────────────────────
# PICO ON/OFF CONTROL (stdin → serial)
# ─────────────────────────────────────────────
# Background thread reads the terminal for 'on' or 'off' and forwards the
# command to the Pico over serial.  Non-blocking relative to the camera loop.

def _pico_control_thread():
    """Read stdin for 'on'/'off' and send to Pico.  Runs as a daemon thread."""
    print("[CONTROL] Type 'on' or 'off' to start/stop the Pico. (default: off)")
    while True:
        try:
            cmd = input().strip().lower()
        except EOFError:
            break
        if cmd in ("on", "off"):
            print(f"[CONTROL] Sending '{cmd}' to Pico")
            send_command(cmd)
        else:
            print(f"[CONTROL] Unknown: '{cmd}' — use 'on' or 'off'")

threading.Thread(target=_pico_control_thread, daemon=True).start()

# ─────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────

print("[CAMERA] Starting posture detection... Press Ctrl+C to quit.\n")

# Shared state between capture loop (main thread) and pose worker thread.
# Main thread is fast (capture + draw + stream push). Worker thread runs
# MediaPipe on whatever raw frame is latest — stream no longer waits on it.
pose_lock = threading.Lock()
pose_raw_frame = None          # latest BGR frame awaiting inference
pose_raw_id = 0                # monotonic id so worker skips stale frames
pose_state = {
    "cached_landmarks": None,
    "stable_slouching": False,
    "bad_since": None,
    "last_reasons": set(),
    "person_lost_streak": 0,
    "aiming_mode": False,
    "_last_id": 0,
}
pose_stop = threading.Event()


def pose_worker():
    global fired
    while not pose_stop.is_set():
        with pose_lock:
            if pose_raw_frame is None or pose_raw_id == pose_state["_last_id"]:
                frame = None
            else:
                frame = pose_raw_frame
                pose_state["_last_id"] = pose_raw_id
        if frame is None:
            time.sleep(0.01)
            continue

        # Downscale for MediaPipe only — the model's input is ~256 sq anyway,
        # so smaller input just cuts the preprocess cost. Landmarks are
        # normalized (0-1), so they map back to any frame size for drawing.
        h, w = frame.shape[:2]
        small = cv2.resize(frame, (128, int(128 * h / w))) if w > 128 else frame
        rgb_frame = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        t0 = time.time()
        results = pose.process(rgb_frame)
        now = time.time()

        with pose_lock:
            if results.pose_landmarks:
                pose_state["person_lost_streak"] = 0
                lm = results.pose_landmarks
                pose_state["cached_landmarks"] = lm
                raw_bad, reasons = evaluate_posture(lm)
                pose_state["last_reasons"] = reasons

                if raw_bad:
                    if pose_state["bad_since"] is None:
                        pose_state["bad_since"] = now
                        print("[POSTURE] WARNING")
                    hold = now - pose_state["bad_since"]
                    pose_state["stable_slouching"] = hold >= BAD_HOLD_SECONDS
                else:
                    if pose_state["bad_since"] is not None:
                        print("[POSTURE] GOOD")
                    pose_state["bad_since"] = None
                    pose_state["stable_slouching"] = False
                    pose_state["last_reasons"] = set()

                if pose_state["stable_slouching"]:
                    if not pose_state["aiming_mode"]:
                        print("[POSTURE] BAD")
                        print("[AIM] Aiming...")
                        pose_state["aiming_mode"] = True
                        fired = False
                    fw_local = frame.shape[1]
                    handle_aiming(lm, fw_local)
                else:
                    if pose_state["aiming_mode"]:
                        pose_state["aiming_mode"] = False
                        fired = False
                        stop_motor()
            else:
                pose_state["person_lost_streak"] += 1
                if pose_state["person_lost_streak"] >= PERSON_LOST_STREAK:
                    pose_state["cached_landmarks"] = None
                    pose_state["stable_slouching"] = False
                    pose_state["bad_since"] = None
                    pose_state["last_reasons"] = set()
                    if pose_state["aiming_mode"]:
                        print("[POSTURE] GOOD")
                        pose_state["aiming_mode"] = False
                        fired = False
                        stop_motor()

        # Idle gap between inferences. Raising this lowers the inference
        # core's duty cycle (cools the Pi) at the cost of landmark Hz.
        # 90 ms + ~285 ms/inference ≈ 76% duty on the hot core.
        time.sleep(0.09)


threading.Thread(target=pose_worker, daemon=True).start()


try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[CAMERA] Read failed")
            break

        # Match requested size so MediaPipe norms line up with drawing (drivers often ignore WxH).
        if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

        with pose_lock:
            pose_raw_frame = frame
            pose_raw_id += 1
            cached_landmarks = pose_state["cached_landmarks"]
            stable_slouching = pose_state["stable_slouching"]
            bad_since = pose_state["bad_since"]
            last_reasons = pose_state["last_reasons"]

        display_frame = frame.copy()

        if cached_landmarks is not None:
            overlay_reasons = last_reasons if stable_slouching else set()
            draw_posture_overlay(display_frame, cached_landmarks, stable_slouching, overlay_reasons)

            if stable_slouching:
                status = "BAD POSTURE"
                color = (0, 0, 255)
            elif bad_since is not None:
                hold = time.time() - bad_since
                remaining = max(0.0, BAD_HOLD_SECONDS - hold)
                status = f"POSTURE WARNING ({remaining:.1f}s hold)"
                color = (0, 165, 255)
            else:
                status = "GOOD POSTURE"
                color = (0, 255, 0)
            cv2.putText(
                display_frame,
                status,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                color,
                2,
            )
        else:
            cv2.putText(
                display_frame,
                "NO PERSON DETECTED",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
            )

        latest_frame = display_frame

except KeyboardInterrupt:
    print("\n[INFO] Stopped.")
finally:
    pose_stop.set()
    try:
        cap.release()
    except Exception:
        pass
    if ser:
        try:
            ser.close()
        except Exception:
            pass
