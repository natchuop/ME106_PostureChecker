import cv2
import mediapipe as mp

mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

pose = mp_pose.Pose(model_complexity=0, enable_segmentation=False)
hands = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.6, min_tracking_confidence=0.6)

cap = cv2.VideoCapture(1, cv2.CAP_MSMF)  # change to CAP_DSHOW if that worked for you
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    raise RuntimeError("Camera failed to open. Try changing index (0->1) or backend (MSMF<->DSHOW).")

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        # If you see this repeatedly, it’s still a camera/backend issue
        continue

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    pose_results = pose.process(rgb)
    hand_results = hands.process(rgb)

    h, w, _ = frame.shape

    # Draw ONLY shoulders/elbows/wrists + arm lines (no face)
    if pose_results.pose_landmarks:
        lm = pose_results.pose_landmarks.landmark

        def pt(p):
            return int(lm[p].x * w), int(lm[p].y * h)

        Ls, Le, Lw = pt(mp_pose.PoseLandmark.LEFT_SHOULDER), pt(mp_pose.PoseLandmark.LEFT_ELBOW), pt(mp_pose.PoseLandmark.LEFT_WRIST)
        Rs, Re, Rw = pt(mp_pose.PoseLandmark.RIGHT_SHOULDER), pt(mp_pose.PoseLandmark.RIGHT_ELBOW), pt(mp_pose.PoseLandmark.RIGHT_WRIST)

        for p in [Ls, Le, Lw, Rs, Re, Rw]:
            cv2.circle(frame, p, 7, (0, 255, 0), -1)

        cv2.line(frame, Ls, Le, (0, 255, 0), 3)
        cv2.line(frame, Le, Lw, (0, 255, 0), 3)
        cv2.line(frame, Rs, Re, (0, 255, 0), 3)
        cv2.line(frame, Re, Rw, (0, 255, 0), 3)

    # Draw hands skeleton
    if hand_results.multi_hand_landmarks:
        for hand_landmarks in hand_results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    cv2.imshow("Arm + Hands Tracking", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()