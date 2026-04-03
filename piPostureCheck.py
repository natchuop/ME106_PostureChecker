"""
zero_main.py - Runs on Raspberry Pi Zero 2 W
- Captures camera feed at low resolution for performance
- Detects slouching using MediaPipe pose detection (lite model)
- Prints direction (LEFT / RIGHT / CENTERED) to terminal
- Streams live video via mjpg-streamer (runs as separate process, zero Python overhead)
- Sends commands to Pico via USB serial

INSTALL REQUIREMENTS:
    sudo apt update
    sudo apt install -y mjpg-streamer python3-pip
    pip install mediapipe opencv-python pyserial

ACCESS STREAM:
    http://<your-pi-ip>:5000/?action=stream
    e.g. http://10.3.141.1:5000/?action=stream  (via RaspAP)
"""

import cv2
import mediapipe as mp
import serial
import time
import subprocess
import os
import signal

# ─────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────

SERIAL_PORT = '/dev/ttyACM0'   # USB serial port to Pico
BAUD_RATE = 9600

FRAME_WIDTH = 320              # Low resolution for performance
FRAME_HEIGHT = 240

FRAME_SKIP = 3                 # Only run pose detection every Nth frame

CENTER_DEADZONE = 20           # Pixel tolerance for "centered" (±20px)

STREAM_PORT = 5000             # mjpg-streamer port
STREAM_FPS = 10                # Stream frame rate (keep low to save CPU)

# Slouch thresholds (tweak these for your setup)
SHOULDER_DROP_THRESHOLD = 0.08
HEAD_FORWARD_THRESHOLD = 0.05

# ─────────────────────────────────────────────
# START MJPG-STREAMER
# ─────────────────────────────────────────────

def start_stream():
    """
    Launch mjpg-streamer as a separate system process.
    This runs entirely outside Python — zero CPU cost to your script.
    """
    cmd = (
        f"mjpg_streamer "
        f"-i 'input_raspicam.so -fps {STREAM_FPS} -x {FRAME_WIDTH} -y {FRAME_HEIGHT}' "
        f"-o 'output_http.so -p {STREAM_PORT} -w /usr/share/mjpg-streamer/www'"
    )
    try:
        proc = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid  # So we can kill the whole process group later
        )
        time.sleep(2)  # Give it a moment to start
        print(f"[STREAM] Live feed at http://<your-pi-ip>:{STREAM_PORT}/?action=stream")
        print(f"[STREAM] Via RaspAP: http://10.3.141.1:{STREAM_PORT}/?action=stream")
        return proc
    except Exception as e:
        print(f"[STREAM] Could not start mjpg-streamer: {e}")
        print("[STREAM] Make sure it's installed: sudo apt install mjpg-streamer")
        return None

def stop_stream(proc):
    """Kill mjpg-streamer process group cleanly."""
    if proc:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            print("[STREAM] Stream stopped.")
        except Exception:
            pass

# ─────────────────────────────────────────────
# SETUP
# ─────────────────────────────────────────────

# Start stream first before loading heavy ML model
stream_proc = start_stream()

# Serial connection to Pico
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"[SERIAL] Connected to Pico on {SERIAL_PORT}")
except Exception as e:
    print(f"[SERIAL] Warning: Could not connect to Pico: {e}")
    ser = None

# Camera (for pose detection only — mjpg-streamer handles its own camera feed)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, 15)

# MediaPipe pose - lite model for performance
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(
    model_complexity=0,
    smooth_landmarks=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# ─────────────────────────────────────────────
# SERIAL HELPERS
# ─────────────────────────────────────────────

def send_command(cmd):
    """Send a command string to the Pico over serial."""
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
# SLOUCH DETECTION
# ─────────────────────────────────────────────

def is_slouching(landmarks):
    """
    Returns True if slouching:
    - Shoulders have dropped below threshold
    - Head has dropped forward relative to shoulders
    """
    lm = landmarks.landmark

    nose = lm[mp_pose.PoseLandmark.NOSE]
    left_shoulder = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
    right_shoulder = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]

    shoulder_y = (left_shoulder.y + right_shoulder.y) / 2
    shoulders_dropped = shoulder_y > (0.5 + SHOULDER_DROP_THRESHOLD)
    head_forward = nose.y > (shoulder_y - HEAD_FORWARD_THRESHOLD)

    return shoulders_dropped and head_forward

# ─────────────────────────────────────────────
# AIMING
# ─────────────────────────────────────────────

def get_person_center_x(landmarks):
    """Returns horizontal center of person in pixels (shoulder midpoint)."""
    lm = landmarks.landmark
    left_shoulder = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
    right_shoulder = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    center_x_norm = (left_shoulder.x + right_shoulder.x) / 2
    return int(center_x_norm * FRAME_WIDTH)

fired = False

def handle_aiming(landmarks):
    """Send motor commands based on how far off-center the person is."""
    global fired

    frame_center = FRAME_WIDTH // 2
    person_x = get_person_center_x(landmarks)
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

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[CAMERA] Read failed")
            break

        frame_count += 1

        # Skip frames for performance
        if frame_count % FRAME_SKIP != 0:
            continue

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(rgb_frame)

        if results.pose_landmarks:
            slouching = is_slouching(results.pose_landmarks)

            if slouching:
                if not aiming_mode:
                    print("[POSTURE] Slouch detected — entering aiming mode...")
                    aiming_mode = True
                    fired = False
                handle_aiming(results.pose_landmarks)
            else:
                if aiming_mode:
                    print("[POSTURE] Good posture restored — exiting aiming mode.")
                    aiming_mode = False
                    fired = False
                    stop_motor()
        else:
            if aiming_mode:
                stop_motor()

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n[INFO] Stopped.")
finally:
    cap.release()
    if ser:
        ser.close()
    stop_stream(stream_proc)
