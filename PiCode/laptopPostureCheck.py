"""Posture cam + MJPEG stream (Windows or Pi). pip: mediapipe opencv-python-headless pyserial flask.
Pi CSI: apt install rpicam-apps libopenblas-dev libgl1. Stream: http://<host>:STREAM_PORT/ (8765 Windows default)."""

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

IS_WINDOWS = platform.system() == "Windows"
IS_LINUX = platform.system() == "Linux"

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

SERIAL_PORT = os.environ.get(
    "SERIAL_PORT", "COM7" if IS_WINDOWS else "/dev/ttyACM0"
)
BAUD_RATE = 9600
SERIAL_CONNECT_RETRIES = int(os.environ.get("SERIAL_CONNECT_RETRIES", "20"))
SERIAL_RETRY_DELAY_S = float(os.environ.get("SERIAL_RETRY_DELAY_S", "0.6"))

FRAME_WIDTH = 256
FRAME_HEIGHT = 192
PERSON_LOST_STREAK = 4
BAD_HOLD_SECONDS = 3.0
STREAM_PORT = int(os.environ.get("STREAM_PORT", "8765" if IS_WINDOWS else "5000"))
STREAM_FPS = 4
STREAM_JPEG_QUALITY = 35
CAMERA_INDEX = int(os.environ.get("CAMERA_INDEX", "1" if IS_WINDOWS else "0"))

amUsingUltrasonicSensor = 0  # 1 = use Pico ultrasonic lines for banner/range; 0 = not wired

SPINE_ANGLE_MIN_DEG = 156.0
SHOULDER_HIP_HORIZONTAL_MAX = 0.052
FOREHEAD_SHOULDER_CLEARANCE_MIN = 0.118
TRAP_NECK_CLEARANCE_MIN = 0.058
CHIN_BELOW_EAR_Y = 0.045
HEAD_OFF_CENTER_X_MAX = 0.068
SHOULDER_SLOPE_MAX_DEG = 5.5
EAR_SLOPE_MAX_DEG = 6.5
HIP_MIN_VISIBILITY = 0.35
NOSE_MIN_VISIBILITY = 0.2
EAR_MIN_VISIBILITY = 0.22
FOREHEAD_VIS_MIN = 0.18

AIM_DEADBAND_PX = 7
FLYWHEEL_CYCLE_TIME_MS = 10000

ser = None
serial_lock = threading.Lock()


def _open_serial_once():
    return serial.Serial(
        SERIAL_PORT,
        BAUD_RATE,
        timeout=0.1,
        write_timeout=1.0,
        rtscts=False,
        dsrdtr=False,
        xonxoff=False,
    )


def _boot_pico_main():
    with serial_lock:
        ser.reset_input_buffer()
        ser.write(b"\x02")
    time.sleep(0.2)
    with serial_lock:
        ser.write(b"\x03")
    time.sleep(0.4)
    with serial_lock:
        ser.write(b"\x04")
    time.sleep(3.0)


for attempt in range(1, SERIAL_CONNECT_RETRIES + 1):
    try:
        ser = _open_serial_once()
        print(f"[SERIAL] Connected to Pico on {SERIAL_PORT}")
        _boot_pico_main()
        break
    except Exception as e:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
            ser = None
        if attempt < SERIAL_CONNECT_RETRIES:
            print(
                f"[SERIAL] Waiting for Pico on {SERIAL_PORT} "
                f"(attempt {attempt}/{SERIAL_CONNECT_RETRIES}): {e}"
            )
            time.sleep(SERIAL_RETRY_DELAY_S)
        else:
            print(
                f"[SERIAL] No Pico on {SERIAL_PORT} after {SERIAL_CONNECT_RETRIES} attempts "
                f"({e}). Continuing without motor commands."
            )

posture_tracking_enabled = threading.Event()


def _frame_brightness(frame):
    if frame is None or frame.size == 0:
        return 0.0
    return float(np.mean(frame))


class PiCamMJPEG:
    """CSI camera via rpicam-vid MJPEG pipe; read() / release() / isOpened() only."""

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
    """Open a non-black capture device (CSI via rpicam on Pi; MSMF/DSHOW on Windows)."""
    if IS_LINUX and _has_csi_camera():
        try:
            print("[CAMERA] CSI Pi Camera detected — using rpicam-vid (MJPEG)")
            pi = PiCamMJPEG(FRAME_WIDTH, FRAME_HEIGHT, fps=6)
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
        backends = [
            (cv2.CAP_MSMF, "Media Foundation"),
            (cv2.CAP_DSHOW, "DirectShow"),
        ]
    elif IS_LINUX:
        backends = [
            (cv2.CAP_V4L2, "V4L2"),
            (None, "default"),
        ]
    else:
        backends = [(None, "default")]

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
        print("  • USB webcams work out of the box via V4L2; CSI cameras need libcamera/rpicam")
        print("    (install with: sudo apt install rpicam-apps)")
    sys.exit(1)

pose = mp_pose.Pose(
    model_complexity=0,
    smooth_landmarks=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)

latest_frame = None


def _discover_lan_ip():
    """Best-effort LAN IP for stream URL (UDP trick)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def start_stream():
    from flask import Flask, Response

    app = Flask(__name__)
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
    time.sleep(0.2)


start_stream()


def _serial_always_send_raw(cmd: str) -> bool:
    c = (cmd or "").strip().lower()
    if c in ("home", "h", "on", "off", "stop", "ultrasonic"):
        return True
    if c.startswith("p,"):
        return True
    try:
        float(c)
        return True
    except ValueError:
        return False


def send_command(cmd):
    if not ser:
        if not getattr(send_command, "_warned_none", False):
            print("[SERIAL] No Pico serial — commands are not sent. Set SERIAL_PORT or connect USB.")
            send_command._warned_none = True
        return
    try:
        now = time.time()
        if not hasattr(send_command, "_last_other_cmd"):
            send_command._last_other_cmd = None
            send_command._last_other_time = 0.0

        if not _serial_always_send_raw(cmd):
            if cmd == send_command._last_other_cmd and (now - send_command._last_other_time) < 0.15:
                return
            send_command._last_other_cmd = cmd
            send_command._last_other_time = now
        else:
            send_command._last_other_cmd = cmd
            send_command._last_other_time = now

        data = f"{cmd}\n".encode()
        with serial_lock:
            ser.write(data)
            try:
                ser.flush()
            except Exception:
                pass
    except Exception as e:
        print(f"[SERIAL] Error: {e}")


def _serial_reader_thread():
    buf = b""
    while True:
        if not ser:
            time.sleep(0.2)
            continue
        try:
            with serial_lock:
                n = ser.in_waiting
                if n:
                    buf += ser.read(n)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    print(f"[PICO] {text}")
                    low = text.lower()
                    if "home set:" in low:
                        posture_tracking_enabled.set()
                        print("[HOMING] Posture tracking enabled.")
                        send_command("on")
                        print("[CONTROL] Sent 'on' — motors activated.")
                    elif "entering home mode" in low:
                        posture_tracking_enabled.clear()
                        print("[HOMING] Posture tracking paused — camera still live.")
                        with pose_lock:
                            pose_state["cached_landmarks"] = None
                            pose_state["stable_slouching"] = False
                            pose_state["bad_since"] = None
                            pose_state["last_reasons"] = set()
                            pose_state["person_lost_streak"] = 0
                    elif amUsingUltrasonicSensor and text == "USONIC_WEB:out_of_range":
                        with pose_lock:
                            pose_state["ultrasonic_banner"] = "TARGET OUT OF RANGE"
                    elif amUsingUltrasonicSensor and text == "USONIC_WEB:in_range":
                        with pose_lock:
                            pose_state["ultrasonic_banner"] = None
        except Exception:
            pass
        time.sleep(0.01)


plm = mp_pose.PoseLandmark


def spine_angle_deg(lm):
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
    dx = abs(bx - ax)
    dy = abs(by - ay)
    if dx < 1e-6:
        return 90.0
    return float(np.degrees(np.arctan2(dy, dx)))


def evaluate_posture(landmarks):
    """Return (is_bad, reasons). Strong side/torso cues alone => bad; else front cues (slope/tilt/off-center)."""
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

    _draw_dense_on_connections(frame, lm, w, h, TORSO_CONNECTIONS, steps=4)

    for idx in TORSO_JOINT_INDICES:
        p = lm[idx]
        if getattr(p, "visibility", 1.0) < 0.2:
            continue
        if idx in (plm.LEFT_SHOULDER, plm.RIGHT_SHOULDER) and slouching and "shoulder_slope" in reasons:
            c = (0, 0, 255)
        elif idx in (plm.LEFT_HIP, plm.RIGHT_HIP) and slouching and "torso_lean" in reasons:
            c = (0, 0, 255)
        else:
            c = node_bgr
        cv2.circle(frame, _pt_xy(lm, idx, w, h), 5, c, 2)

def _pico_control_thread():
    print("[CONTROL] Type Pico commands (h, on, off, +5, -10, home, …)")
    while True:
        try:
            cmd = input().strip().lower()
        except EOFError:
            break
        if not cmd:
            continue
        print(f"[CONTROL] Sending '{cmd}' to Pico")
        send_command(cmd)

threading.Thread(target=_pico_control_thread, daemon=True).start()

print(
    "[CAMERA] Stream running; posture pauses during Pico homing until [HOMING] enabled. Ctrl+C to quit.\n"
)

pose_lock = threading.Lock()
pose_raw_frame = None
pose_raw_id = 0
pose_state = {
    "cached_landmarks": None,
    "stable_slouching": False,
    "bad_since": None,
    "last_reasons": set(),
    "person_lost_streak": 0,
    "steady_bad_announced": False,
    "_last_id": 0,
    "ultrasonic_banner": None,
    "fire_cooldown_until": 0.0,
}
pose_stop = threading.Event()
camera_shutdown = threading.Event()


def pose_worker():
    while not pose_stop.is_set():
        if not posture_tracking_enabled.is_set():
            time.sleep(0.05)
            continue
        with pose_lock:
            if pose_raw_frame is None or pose_raw_id == pose_state["_last_id"]:
                frame = None
            else:
                frame = pose_raw_frame
                pose_state["_last_id"] = pose_raw_id
        if frame is None:
            time.sleep(0.01)
            continue

        h, w = frame.shape[:2]
        small = cv2.resize(frame, (128, int(128 * h / w))) if w > 128 else frame
        rgb_frame = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
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
                    pose_state["fire_cooldown_until"] = 0.0

                if pose_state["stable_slouching"]:
                    if not pose_state["steady_bad_announced"]:
                        print("[POSTURE] BAD")
                        pose_state["steady_bad_announced"] = True
                else:
                    pose_state["steady_bad_announced"] = False
            else:
                pose_state["person_lost_streak"] += 1
                if pose_state["person_lost_streak"] >= PERSON_LOST_STREAK:
                    pose_state["cached_landmarks"] = None
                    pose_state["stable_slouching"] = False
                    pose_state["bad_since"] = None
                    pose_state["last_reasons"] = set()
                    if pose_state["steady_bad_announced"]:
                        print("[POSTURE] GOOD")
                    pose_state["steady_bad_announced"] = False

        # Platform aim only in sustained bad posture ("aiming mode"); otherwise idle the turntable.
        if posture_tracking_enabled.is_set():
            with pose_lock:
                stable_bad = pose_state["stable_slouching"]
                fire_cooldown_until = pose_state["fire_cooldown_until"]
            should_aim = bool(results.pose_landmarks and stable_bad)
            if should_aim:
                _lm = results.pose_landmarks.landmark
                _mid_x = (_lm[plm.LEFT_SHOULDER].x + _lm[plm.RIGHT_SHOULDER].x) * 0.5
                _error_px = int((_mid_x - 0.5) * FRAME_WIDTH)
                
                now = time.time()
                # Only send P commands if not in cooldown
                if now < fire_cooldown_until:
                    # During cooldown, don't move platform
                    pass
                else:
                    # Send fixed-speed commands: P -20 or P 20
                    if _error_px < 0:
                        send_command("P,-20")
                    elif _error_px > 0:
                        send_command("P,20")
                    else:
                        send_command("stop")
                    
                    # Fire when centered (within deadband) and cooldown expired
                    if abs(_error_px) <= AIM_DEADBAND_PX:
                        send_command("fire")
                        with pose_lock:
                            pose_state["fire_cooldown_until"] = now + (FLYWHEEL_CYCLE_TIME_MS / 1000.0)
                        print("[FIRING] Centered and firing!")
            else:
                if not results.pose_landmarks:
                    send_command("stop")
                elif getattr(pose_worker, "_prev_should_aim", False):
                    send_command("stop")
            pose_worker._prev_should_aim = should_aim

        time.sleep(0.09)

threading.Thread(target=pose_worker, daemon=True).start()


def _camera_capture_loop():
    global latest_frame, pose_raw_frame, pose_raw_id
    while not camera_shutdown.is_set():
        ret, frame = cap.read()
        if not ret:
            print("[CAMERA] Read failed")
            break

        if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

        with pose_lock:
            pose_raw_frame = frame
            pose_raw_id += 1
            cached_landmarks = pose_state["cached_landmarks"]
            stable_slouching = pose_state["stable_slouching"]
            bad_since = pose_state["bad_since"]
            last_reasons = pose_state["last_reasons"]
            ultra_banner = (
                pose_state["ultrasonic_banner"]
                if amUsingUltrasonicSensor
                else None
            )

        display_frame = frame.copy()
        tracking = posture_tracking_enabled.is_set()

        if not tracking:
            cv2.putText(
                display_frame,
                "HOMING IN PROGRESS",
                (8, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.64,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        elif cached_landmarks is not None:
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

        if ultra_banner:
            cv2.putText(
                display_frame,
                ultra_banner,
                (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 140, 255),
                2,
                cv2.LINE_AA,
            )

        latest_frame = display_frame


cam_thread = threading.Thread(target=_camera_capture_loop, daemon=True)
cam_thread.start()
time.sleep(0.3)

if ser:
    threading.Thread(target=_serial_reader_thread, daemon=True).start()
    send_command("h")
    print("[CONTROL] Entering homing mode. Jog the platform and type 'home' in the Pico terminal to center it.")
else:
    print("[SERIAL] No Pico detected — posture tracking disabled.")

try:
    while True:
        time.sleep(0.25)
except KeyboardInterrupt:
    print("\n[INFO] Stopped (Ctrl+C).")
finally:
    pose_stop.set()
    camera_shutdown.set()
    if ser:
        try:
            send_command("stop")
            time.sleep(0.08)
            send_command("off")
        except Exception:
            pass
    cam_thread.join(timeout=3.0)
    try:
        cap.release()
    except Exception:
        pass
    if ser:
        try:
            ser.close()
        except Exception:
            pass
