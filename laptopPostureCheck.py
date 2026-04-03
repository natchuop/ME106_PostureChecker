"""
zero_main.py - Runs on Raspberry Pi Zero 2 W
- Captures camera feed at low resolution for performance
- Detects slouching using MediaPipe pose detection (lite model)
- Prints direction (LEFT / RIGHT / CENTERED) to terminal
- Streams live video (mjpg-streamer on Pi, Flask fallback on Windows)
- Sends commands to Pico via USB serial

INSTALL REQUIREMENTS (Pi):
    sudo apt update
    sudo apt install -y mjpg-streamer python3-pip
    pip install mediapipe opencv-python pyserial flask

INSTALL REQUIREMENTS (Windows/testing):
    pip install mediapipe opencv-python pyserial flask

ACCESS STREAM:
    Pi:      http://10.3.141.1:5000/?action=stream
    Windows: http://127.0.0.1:8765 (default; set STREAM_PORT if needed)
"""

import cv2
import numpy as np
import serial
import time
import subprocess
import os
import sys
import platform
import threading

# ─────────────────────────────────────────────
# DETECT PLATFORM
# ─────────────────────────────────────────────

IS_WINDOWS = platform.system() == "Windows"
DEBUG_MODE = IS_WINDOWS  # Automatically use debug mode on Windows

if DEBUG_MODE:
    print("[INFO] Running in DEBUG MODE (Windows) — using Flask stream, no serial/mjpg-streamer")

# ─────────────────────────────────────────────
# MEDIAPIPE IMPORT (Windows vs Pi)
# ─────────────────────────────────────────────

try:
    import mediapipe as mp
    mp_pose = mp.solutions.pose
    print("[INFO] MediaPipe loaded successfully")
except Exception as e:
    print(f"[ERROR] MediaPipe failed to load: {e}")
    print("Run: pip install mediapipe")
    sys.exit(1)

# ─────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────

SERIAL_PORT = 'COM3' if IS_WINDOWS else '/dev/ttyACM0'
BAUD_RATE = 9600

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
# Pose runs every Nth frame (1 = every frame). Overlay uses cached landmarks every frame.
FRAME_SKIP = 2
CENTER_DEADZONE = 20
# Require consecutive pose samples before toggling slouch state (reduces flicker).
SLOUCH_ON_STREAK = 4
SLOUCH_OFF_STREAK = 5
# Drop "person lost" only after this many misses (brief occlusions recover).
PERSON_LOST_STREAK = 4
# Confirm bad posture only after holding it for this long (seconds).
BAD_HOLD_SECONDS = 3.0
# Windows often has port 5000 taken (AirPlay / other services); override with STREAM_PORT=5000 if needed.
STREAM_PORT = int(os.environ.get("STREAM_PORT", "8765" if IS_WINDOWS else "5000"))
STREAM_FPS = 10

# Webcam index for this machine (edit here only — not overridden elsewhere)
CAMERA_INDEX = 1

# Posture (normalized 0..1, y grows downward). Webcam = mostly frontal; 2D proxies for seated cues.
SHOULDER_BROAD_MIN = 0.15
ELBOW_BROAD_MIN = 0.13
# Head / neck vs shoulders
SPINE_ANGLE_MIN_DEG = 156.0              # more forgiving: allow a bit more forward bend
NECK_EAR_ANGLE_MIN_DEG = 150.0           # ear–mid_shoulder–mid_hip; neck too folded vs torso
FOREHEAD_SHOULDER_CLEARANCE_MIN = 0.118
NOSE_SHOULDER_CLEARANCE_MIN = 0.085
TRAP_NECK_CLEARANCE_MIN = 0.058          # ears too close vertically to shoulders
CHIN_BELOW_EAR_Y = 0.045                 # more forgiving chin down
HEAD_OFF_CENTER_X_MAX = 0.068          # |nose.x − mid_shoulder.x| head tilted or shifted off center
# Shoulders / torso / hips / symmetry
SHOULDER_Y_ASYMMETRY_MAX = 0.042       # |L_shoulder.y − R_shoulder.y| uneven shoulders
SHOULDER_HIP_HORIZONTAL_MAX = 0.052    # |mid_shoulder.x − mid_hip.x| lean or shoulders forward of hips (2D)
TORSO_COLLAPSE_MAX = 0.095
HIP_MIN_VISIBILITY = 0.35
NOSE_MIN_VISIBILITY = 0.2
EAR_MIN_VISIBILITY = 0.22
EAR_Y_ASYMMETRY_MAX = 0.035      # abs(left_ear.y - right_ear.y) for head tilt / uneven head
FOREHEAD_VIS_MIN = 0.18

# ─────────────────────────────────────────────
# SERIAL SETUP
# ─────────────────────────────────────────────

ser = None
if not DEBUG_MODE:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"[SERIAL] Connected to Pico on {SERIAL_PORT}")
    except Exception as e:
        print(f"[SERIAL] Warning: Could not connect to Pico: {e}")

# ─────────────────────────────────────────────
# CAMERA SETUP
# ─────────────────────────────────────────────

def _frame_brightness(frame):
    """Mean pixel level; ~0 = black (wrong/disabled device), higher = real picture."""
    if frame is None or frame.size == 0:
        return 0.0
    return float(np.mean(frame))


def open_camera():
    """
    Find a capture device that actually delivers non-black frames.
    On Windows, try Media Foundation before DirectShow — DSHOW often opens a
    phantom/virtual device that reads OK but is all black.
    """
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
        print("  • Quit other apps using the camera; edit CAMERA_INDEX at top of this file")
        print("  • Integrated vs USB webcam: try another index until brightness looks right in the console")
    sys.exit(1)

# ─────────────────────────────────────────────
# MEDIAPIPE POSE SETUP
# ─────────────────────────────────────────────

pose = mp_pose.Pose(
    model_complexity=0,
    smooth_landmarks=True,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.65,
)

# ─────────────────────────────────────────────
# STREAMING
# ─────────────────────────────────────────────

latest_frame = None
stream_proc = None

def start_mjpg_stream():
    """Launch mjpg-streamer on Pi (Linux only)."""
    cmd = (
        f"mjpg_streamer "
        f"-i 'input_raspicam.so -fps {STREAM_FPS} -x {FRAME_WIDTH} -y {FRAME_HEIGHT}' "
        f"-o 'output_http.so -p {STREAM_PORT} -w /usr/share/mjpg-streamer/www'"
    )
    try:
        import signal
        proc = subprocess.Popen(
            cmd, shell=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        time.sleep(2)
        print(f"[STREAM] Live feed at http://10.3.141.1:{STREAM_PORT}/?action=stream")
        return proc
    except Exception as e:
        print(f"[STREAM] Could not start mjpg-streamer: {e}")
        return None

def start_flask_stream():
    """Launch lightweight Flask stream for Windows/debug."""
    from flask import Flask, Response

    app = Flask(__name__)

    def encode_jpeg(frame):
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
        if not ok or buf is None:
            return None
        return buf.tobytes()

    @app.route("/frame.jpg")
    def frame_jpg():
        """Single-frame JPEG; page polls this. Encode outside any lock — main only swaps `latest_frame` ref."""
        f = latest_frame
        if f is None:
            f = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
        data = encode_jpeg(f.copy())
        if data is None:
            return Response(status=503)
        return Response(
            data,
            mimetype="image/jpeg",
            headers={
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
            },
        )

    @app.route('/')
    def index():
        return '''
        <html>
        <head>
            <title>Posture Cam</title>
            <style>
                body { background:#111; color:#eee; font-family:sans-serif; text-align:center; padding:20px; }
                img { border:2px solid #444; border-radius:8px; max-width:100%; height:auto; }
            </style>
        </head>
        <body>
            <h2>Posture Cam - Live Feed</h2>
            <img id="cam" width="640" height="480" alt="camera feed" />
            <script>
            (function(){
                var img = document.getElementById('cam');
                var n = 0;
                function tick() {
                    img.src = '/frame.jpg?n=' + (++n) + '&t=' + Date.now();
                }
                tick();
                setInterval(tick, 40);
            })();
            </script>
        </body>
        </html>'''

    import socket
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    print(f"[STREAM] Live feed at http://{local_ip}:{STREAM_PORT}")
    print(f"[STREAM] Open in browser: http://127.0.0.1:{STREAM_PORT}")

    thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=STREAM_PORT, threaded=True, use_reloader=False),
        daemon=True
    )
    thread.start()
    time.sleep(0.15)  # Let the dev server bind before the main loop / browser polls /frame.jpg

# Start appropriate stream
if DEBUG_MODE:
    start_flask_stream()
else:
    stream_proc = start_mjpg_stream()

# ─────────────────────────────────────────────
# SERIAL HELPERS
# ─────────────────────────────────────────────

def send_command(cmd):
    if ser:
        try:
            ser.write(f"{cmd}\n".encode())
        except Exception as e:
            print(f"[SERIAL] Error: {e}")

def rotate_left(error):
    print(f"[AIM] Target is to the LEFT  | error: {error}px")
    send_command(f"move:{error}")

def rotate_right(error):
    print(f"[AIM] Target is to the RIGHT | error: +{error}px")
    send_command(f"move:{error}")

def stop_motor():
    send_command("stop")

def fire():
    print("[FIRE] Target centered — FIRING!")
    send_command("fire")

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


def _angle_at_vertex_deg(ax, ay, bx, by, cx, cy):
    """Angle at vertex B between BA and BC (degrees)."""
    v1 = np.array([ax - bx, ay - by], dtype=np.float64)
    v2 = np.array([cx - bx, cy - by], dtype=np.float64)
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return None
    c = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


def neck_ear_torso_angle_deg(lm, mid_sx, mid_sy, mid_hx, mid_hy):
    """Neck/torso chain using ears as head anchor (angle at mid-shoulder)."""
    lea = lm[plm.LEFT_EAR]
    rea = lm[plm.RIGHT_EAR]
    if (
        getattr(lea, "visibility", 1.0) < EAR_MIN_VISIBILITY
        or getattr(rea, "visibility", 1.0) < EAR_MIN_VISIBILITY
    ):
        return None
    emx = (lea.x + rea.x) * 0.5
    emy = (lea.y + rea.y) * 0.5
    return _angle_at_vertex_deg(emx, emy, mid_sx, mid_sy, mid_hx, mid_hy)


def bad_posture(landmarks):
    """
    Split into two buckets (both from a single mostly-front webcam):
    Side-view proxies (forward head, neck folding, spine curvature, torso collapse/lean):
      - spine angle (nose–shoulder–hip)
      - forehead/nose clearance vs shoulders
      - trap/neck compression from ears
      - chin-down vs ear line
      - neck angle (ear midpoint → shoulder midpoint → hip midpoint)
      - torso collapse (only when chest isn't broad)
    Front-view proxies (asymmetry):
      - shoulder height imbalance
      - ear height imbalance (head tilt)
      - nose off-center over shoulders
      - shoulder/hip horizontal shift (leans to one side)

    Rule: if any side-view proxy is bad => slouch.
    Otherwise, front-view must have 2+ issues OR a shoulder-height imbalance.
    Broad chest/elbows cancels only torso-collapse (not head/neck/head-position).
    """
    lm = landmarks.landmark
    nose = lm[plm.NOSE]
    ls = lm[plm.LEFT_SHOULDER]
    rs = lm[plm.RIGHT_SHOULDER]
    le = lm[plm.LEFT_ELBOW]
    re = lm[plm.RIGHT_ELBOW]
    lh = lm[plm.LEFT_HIP]
    rh = lm[plm.RIGHT_HIP]

    mid_sx = (ls.x + rs.x) * 0.5
    mid_sy = (ls.y + rs.y) * 0.5
    mid_hx = (lh.x + rh.x) * 0.5
    mid_hy = (lh.y + rh.y) * 0.5

    hip_vis = min(
        getattr(lh, "visibility", 1.0),
        getattr(rh, "visibility", 1.0),
    )
    nose_vis = getattr(nose, "visibility", 1.0)

    shoulder_w = abs(ls.x - rs.x)
    elbow_w = abs(le.x - re.x)
    broad = shoulder_w >= SHOULDER_BROAD_MIN or elbow_w >= ELBOW_BROAD_MIN

    ears_ok = False
    lea = lm[plm.LEFT_EAR]
    rea = lm[plm.RIGHT_EAR]
    if (
        getattr(lea, "visibility", 1.0) >= EAR_MIN_VISIBILITY
        and getattr(rea, "visibility", 1.0) >= EAR_MIN_VISIBILITY
    ):
        ears_ok = True

    # -------------------------
    # Side-view badness
    # -------------------------
    side_bad = False

    ang = spine_angle_deg(lm)
    if (
        ang is not None
        and nose_vis >= NOSE_MIN_VISIBILITY
        and hip_vis >= HIP_MIN_VISIBILITY
        and ang < SPINE_ANGLE_MIN_DEG
    ):
        side_bad = True

    face_top_y = _face_top_y(lm)
    if face_top_y is not None and (mid_sy - face_top_y) < FOREHEAD_SHOULDER_CLEARANCE_MIN:
        side_bad = True

    if nose_vis >= NOSE_MIN_VISIBILITY and (mid_sy - nose.y) < NOSE_SHOULDER_CLEARANCE_MIN:
        side_bad = True

    if ears_ok:
        ear_mid_y = (lea.y + rea.y) * 0.5
        if (mid_sy - ear_mid_y) < TRAP_NECK_CLEARANCE_MIN:
            side_bad = True
        if nose_vis >= NOSE_MIN_VISIBILITY and (nose.y - ear_mid_y) > CHIN_BELOW_EAR_Y:
            side_bad = True

    neck_ang = neck_ear_torso_angle_deg(lm, mid_sx, mid_sy, mid_hx, mid_hy)
    if (
        neck_ang is not None
        and hip_vis >= HIP_MIN_VISIBILITY
        and neck_ang < NECK_EAR_ANGLE_MIN_DEG
    ):
        side_bad = True

    # torso lean / hip-to-shoulder relationship proxy (2D lateral shift can happen with forward lean)
    if hip_vis >= HIP_MIN_VISIBILITY and abs(mid_sx - mid_hx) > SHOULDER_HIP_HORIZONTAL_MAX:
        side_bad = True

    # torso collapse: only punish when chest isn't broad/open
    if (not broad) and hip_vis >= HIP_MIN_VISIBILITY and (mid_hy - mid_sy) < TORSO_COLLAPSE_MAX:
        side_bad = True

    if side_bad:
        return True

    # -------------------------
    # Front-view badness
    # -------------------------
    shoulder_asym = abs(ls.y - rs.y) > SHOULDER_Y_ASYMMETRY_MAX
    head_offcenter = nose_vis >= NOSE_MIN_VISIBILITY and abs(nose.x - mid_sx) > HEAD_OFF_CENTER_X_MAX
    ear_asym = ears_ok and abs(lea.y - rea.y) > EAR_Y_ASYMMETRY_MAX
    shift_x = hip_vis >= HIP_MIN_VISIBILITY and abs(mid_sx - mid_hx) > SHOULDER_HIP_HORIZONTAL_MAX

    front_count = sum([shoulder_asym, head_offcenter, ear_asym, shift_x])
    if shoulder_asym:
        return True
    return front_count >= 2


def evaluate_posture(landmarks):
    """
    Like `bad_posture`, but also returns *which* checks failed so we can color the
    specific nodes/lines (head vs neck/traps vs spine/torso vs front asymmetry).
    Returns: (is_bad: bool, reasons: set[str])
    """
    lm = landmarks.landmark
    nose = lm[plm.NOSE]
    ls = lm[plm.LEFT_SHOULDER]
    rs = lm[plm.RIGHT_SHOULDER]
    le = lm[plm.LEFT_ELBOW]
    re = lm[plm.RIGHT_ELBOW]
    lh = lm[plm.LEFT_HIP]
    rh = lm[plm.RIGHT_HIP]

    mid_sx = (ls.x + rs.x) * 0.5
    mid_sy = (ls.y + rs.y) * 0.5
    mid_hx = (lh.x + rh.x) * 0.5
    mid_hy = (lh.y + rh.y) * 0.5

    hip_vis = min(
        getattr(lh, "visibility", 1.0),
        getattr(rh, "visibility", 1.0),
    )
    nose_vis = getattr(nose, "visibility", 1.0)

    shoulder_w = abs(ls.x - rs.x)
    elbow_w = abs(le.x - re.x)
    broad = shoulder_w >= SHOULDER_BROAD_MIN or elbow_w >= ELBOW_BROAD_MIN

    ears_ok = False
    lea = lm[plm.LEFT_EAR]
    rea = lm[plm.RIGHT_EAR]
    if (
        getattr(lea, "visibility", 1.0) >= EAR_MIN_VISIBILITY
        and getattr(rea, "visibility", 1.0) >= EAR_MIN_VISIBILITY
    ):
        ears_ok = True

    reasons = set()

    # -------------------------
    # Side-view badness
    # -------------------------
    side_bad = False

    ang = spine_angle_deg(lm)
    if (
        ang is not None
        and nose_vis >= NOSE_MIN_VISIBILITY
        and hip_vis >= HIP_MIN_VISIBILITY
        and ang < SPINE_ANGLE_MIN_DEG
    ):
        side_bad = True
        reasons.add("spine_angle")

    face_top_y = _face_top_y(lm)
    if face_top_y is not None and (mid_sy - face_top_y) < FOREHEAD_SHOULDER_CLEARANCE_MIN:
        side_bad = True
        reasons.add("forehead_dip")

    if nose_vis >= NOSE_MIN_VISIBILITY and (mid_sy - nose.y) < NOSE_SHOULDER_CLEARANCE_MIN:
        side_bad = True
        reasons.add("nose_dip")

    if ears_ok:
        ear_mid_y = (lea.y + rea.y) * 0.5
        if (mid_sy - ear_mid_y) < TRAP_NECK_CLEARANCE_MIN:
            side_bad = True
            reasons.add("trap_neck")
        if nose_vis >= NOSE_MIN_VISIBILITY and (nose.y - ear_mid_y) > CHIN_BELOW_EAR_Y:
            side_bad = True
            reasons.add("chin_down")

    neck_ang = neck_ear_torso_angle_deg(lm, mid_sx, mid_sy, mid_hx, mid_hy)
    if (
        neck_ang is not None
        and hip_vis >= HIP_MIN_VISIBILITY
        and neck_ang < NECK_EAR_ANGLE_MIN_DEG
    ):
        side_bad = True
        reasons.add("neck_angle")

    # torso lean / hip-to-shoulder relationship proxy (2D lateral shift can happen with forward lean)
    if hip_vis >= HIP_MIN_VISIBILITY and abs(mid_sx - mid_hx) > SHOULDER_HIP_HORIZONTAL_MAX:
        side_bad = True
        reasons.add("torso_lean")

    # torso collapse: only punish when chest isn't broad/open
    if (not broad) and hip_vis >= HIP_MIN_VISIBILITY and (mid_hy - mid_sy) < TORSO_COLLAPSE_MAX:
        side_bad = True
        reasons.add("torso_collapse")

    if side_bad:
        return True, reasons

    # -------------------------
    # Front-view badness (asymmetry)
    # -------------------------
    shoulder_asym = abs(ls.y - rs.y) > SHOULDER_Y_ASYMMETRY_MAX
    head_offcenter = nose_vis >= NOSE_MIN_VISIBILITY and abs(nose.x - mid_sx) > HEAD_OFF_CENTER_X_MAX
    ear_asym = ears_ok and abs(lea.y - rea.y) > EAR_Y_ASYMMETRY_MAX
    shift_x = hip_vis >= HIP_MIN_VISIBILITY and abs(mid_sx - mid_hx) > SHOULDER_HIP_HORIZONTAL_MAX

    if shoulder_asym:
        reasons.add("shoulder_asym")
        return True, reasons

    if head_offcenter:
        reasons.add("head_offcenter")
    if ear_asym:
        reasons.add("ear_asym")
    if shift_x:
        reasons.add("torso_lean")

    front_count = sum([shoulder_asym, head_offcenter, ear_asym, shift_x])
    if front_count >= 2:
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
    (plm.LEFT_SHOULDER, plm.LEFT_ELBOW),
    (plm.RIGHT_SHOULDER, plm.RIGHT_ELBOW),
    (plm.LEFT_SHOULDER, plm.LEFT_HIP),
    (plm.RIGHT_SHOULDER, plm.RIGHT_HIP),
    (plm.LEFT_HIP, plm.RIGHT_HIP),
)
TORSO_JOINT_INDICES = (
    plm.LEFT_SHOULDER,
    plm.RIGHT_SHOULDER,
    plm.LEFT_ELBOW,
    plm.RIGHT_ELBOW,
    plm.LEFT_HIP,
    plm.RIGHT_HIP,
)


def draw_posture_overlay(frame, landmarks, slouching, reasons=None):
    """Bones + neck (nose→shoulders) + spine (mid-shoulder→mid-hip); optional ear→mid-shoulder neck base."""
    if reasons is None:
        reasons = set()
    lm = landmarks.landmark
    h, w = frame.shape[:2]
    head_reasons = {"forehead_dip", "nose_dip", "chin_down", "neck_angle", "trap_neck", "head_offcenter", "ear_asym"}
    spine_reasons = {"spine_angle", "torso_collapse", "torso_lean"}
    trap_reasons = {"trap_neck"}
    shoulder_reasons = {"shoulder_asym"}

    spine_bgr = (0, 0, 255) if (slouching and (reasons & spine_reasons)) else (0, 255, 0)
    neck_bgr = (0, 0, 255) if (slouching and (reasons & head_reasons)) else (0, 200, 255)
    bone_bgr = (40, 200, 255)
    node_bgr = (200, 220, 255)
    head_bgr = (0, 0, 255) if (slouching and (reasons & {"forehead_dip", "nose_dip", "chin_down", "head_offcenter"})) else (200, 200, 255)

    for a, b in TORSO_CONNECTIONS:
        cv2.line(frame, _pt_xy(lm, a, w, h), _pt_xy(lm, b, w, h), bone_bgr, 2)

    ls = _pt_xy(lm, plm.LEFT_SHOULDER, w, h)
    rs = _pt_xy(lm, plm.RIGHT_SHOULDER, w, h)
    lh = _pt_xy(lm, plm.LEFT_HIP, w, h)
    rh = _pt_xy(lm, plm.RIGHT_HIP, w, h)
    mid_s = ((ls[0] + rs[0]) // 2, (ls[1] + rs[1]) // 2)
    mid_h = ((lh[0] + rh[0]) // 2, (lh[1] + rh[1]) // 2)
    nose_pt = _pt_xy(lm, plm.NOSE, w, h)
    trap_bgr = (0, 0, 255) if (slouching and (reasons & trap_reasons)) else ((60, 180, 255) if slouching else (200, 120, 80))

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
        ear_bgr = (0, 0, 255) if (slouching and "ear_asym" in reasons) else head_bgr
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

    ang = spine_angle_deg(lm)
    if ang is not None:
        cv2.putText(
            frame,
            f"spine {ang:.0f}deg",
            (max(4, mid_s[0] - 40), max(16, mid_s[1] - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            spine_bgr,
            1,
            cv2.LINE_AA,
        )

    _draw_dense_on_connections(frame, lm, w, h, TORSO_CONNECTIONS, steps=4)

    for idx in TORSO_JOINT_INDICES:
        p = lm[idx]
        if getattr(p, "visibility", 1.0) < 0.2:
            continue
        # Color important joints based on which check failed.
        if idx in (plm.LEFT_SHOULDER, plm.RIGHT_SHOULDER) and (slouching and (reasons & shoulder_reasons)):
            c = (0, 0, 255)
        elif idx in (plm.LEFT_HIP, plm.RIGHT_HIP) and (slouching and (reasons & {"torso_collapse", "torso_lean"})):
            c = (0, 0, 255)
        else:
            c = node_bgr
        cv2.circle(frame, _pt_xy(lm, idx, w, h), 5, c, 2)

# ─────────────────────────────────────────────
# AIMING
# ─────────────────────────────────────────────

def get_person_center_x(landmarks, w):
    lm = landmarks.landmark
    left_shoulder = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
    right_shoulder = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    center_x_norm = (left_shoulder.x + right_shoulder.x) / 2
    return int(center_x_norm * w)

fired = False

def handle_aiming(landmarks, w):
    global fired
    frame_center = w // 2
    person_x = get_person_center_x(landmarks, w)
    error = person_x - frame_center

    if abs(error) <= CENTER_DEADZONE:
        stop_motor()
        print(f"[AIM] Target CENTERED (error: {error}px)")
        if not fired:
            time.sleep(0.2)
            fire()
            fired = True
    elif error < 0:
        rotate_left(error)
    else:
        rotate_right(error)

# ─────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────

print("[CAMERA] Starting posture detection... Press Ctrl+C to quit.\n")

frame_count = 0
aiming_mode = False

cached_landmarks = None
person_lost_streak = 0
slouch_on_streak = 0
slouch_off_streak = 0
stable_slouching = False

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[CAMERA] Read failed")
            break

        # Match requested size so MediaPipe norms line up with drawing (drivers often ignore WxH).
        if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

        frame_count += 1
        display_frame = frame.copy()
        fw = frame.shape[1]

        if frame_count % FRAME_SKIP == 0:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(rgb_frame)

            if results.pose_landmarks:
                person_lost_streak = 0
                lm = results.pose_landmarks
                cached_landmarks = lm

                raw_slouch = bad_posture(lm)
                if raw_slouch:
                    slouch_off_streak = 0
                    slouch_on_streak += 1
                    if not stable_slouching and slouch_on_streak >= SLOUCH_ON_STREAK:
                        stable_slouching = True
                else:
                    slouch_on_streak = 0
                    slouch_off_streak += 1
                    if stable_slouching and slouch_off_streak >= SLOUCH_OFF_STREAK:
                        stable_slouching = False

                if stable_slouching:
                    if not aiming_mode:
                        print("[POSTURE] Slouch detected — entering aiming mode...")
                        aiming_mode = True
                        fired = False
                    handle_aiming(lm, fw)
                else:
                    if aiming_mode:
                        print("[POSTURE] Good posture restored — exiting aiming mode.")
                        aiming_mode = False
                        fired = False
                        stop_motor()
            else:
                person_lost_streak += 1
                if person_lost_streak >= PERSON_LOST_STREAK:
                    cached_landmarks = None
                    stable_slouching = False
                    slouch_on_streak = 0
                    slouch_off_streak = 0
                    if aiming_mode:
                        aiming_mode = False
                        fired = False
                        stop_motor()

        # Overlay uses same pixel size as frame (landmarks normalized to that image).
        if cached_landmarks is not None:
            draw_posture_overlay(display_frame, cached_landmarks, stable_slouching)
            status = "SLOUCHING" if stable_slouching else "GOOD POSTURE"
            color = (0, 0, 255) if stable_slouching else (0, 255, 0)
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

        # Push to web stream (atomic ref swap; HTTP handler copies before encode)
        latest_frame = display_frame

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\n[INFO] Stopped.")
finally:
    cap.release()
    if ser:
        ser.close()
    if stream_proc:
        try:
            import signal
            os.killpg(os.getpgid(stream_proc.pid), signal.SIGTERM)
        except Exception:
            pass
