import cv2
import socket
import time
from ultralytics import YOLO

# =====================================================
# 1. CONFIGURATION
# =====================================================

CAM_URL = "http://172.20.10.8/stream"
MODEL_PATH = "best.pt"

ROBOT_IP = "172.20.10.9"
ROBOT_PORT = 6868

CONF_THRESHOLD = 0.5
YOLO_INTERVAL = 0.1
SEND_INTERVAL = 1

REAL_WIDTH = 8
FOCAL_LENGTH = 600

RESIZE_WIDTH = 640
RESIZE_HEIGHT = 480

last_send_time = 0

# =====================================================
# 2. TCP CONNECTION
# =====================================================

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    sock.connect((ROBOT_IP, ROBOT_PORT))
    print("✅ Connected to Robot")
except:
    print("❌ Cannot connect to robot")
    sock = None

# =====================================================
# 3. LOAD YOLO MODEL
# =====================================================

model = YOLO(MODEL_PATH)

# =====================================================
# 4. OPEN CAMERA STREAM
# =====================================================

cap = cv2.VideoCapture(CAM_URL)

if not cap.isOpened():
    print("❌ Cannot connect ESP32-CAM")
    exit()

# =====================================================
# 5. GET FRAME SIZE
# =====================================================

ret, first_frame = cap.read()

if not ret:
    print("❌ Cannot read frame")
    exit()

original_h, original_w = first_frame.shape[:2]

scale_w = RESIZE_WIDTH / original_w
scale_h = RESIZE_HEIGHT / original_h
scale = min(scale_w, scale_h)

new_w = int(original_w * scale)
new_h = int(original_h * scale)

FOCAL_LENGTH *= scale

print(f"✅ Camera connected: {original_w}x{original_h} -> {new_w}x{new_h}")

# =====================================================
# 6. VARIABLES
# =====================================================

last_yolo_time = 0
results = None

split_line_y = int(new_h * 0.55)

current_distance = 0

# =====================================================
# 7. MAIN LOOP
# =====================================================

while True:

    ret, frame = cap.read()

    if not ret:
        print("❌ Frame error")
        break

    frame_resized = cv2.resize(frame, (new_w, new_h))

    now = time.time()

    # =============================================
    # YOLO DETECTION
    # =============================================

    if now - last_yolo_time > YOLO_INTERVAL:

        results = model(frame_resized, conf=CONF_THRESHOLD, verbose=False)

        last_yolo_time = now

        current_label = ""
        current_y = 0
        current_distance = 0

        # =============================================
        # PROCESS YOLO RESULT
        # =============================================

        if results:

            for r in results:

                boxes = r.boxes

                for box in boxes:

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls[0])

                    label = model.names[cls]

                    bbox_width = x2 - x1

                    if bbox_width == 0:
                        continue

                    distance = (REAL_WIDTH * FOCAL_LENGTH) / bbox_width

                    # filter obstacle ở nửa trên
                    if label == "rockSign" and y1 < split_line_y:
                        continue

                    current_label = label
                    current_y = y1
                    current_distance = distance

                    # vẽ bounding box
                    color = (0, 0, 255) if label == "rockSign" else (0, 255, 0)

                    cv2.rectangle(
                        frame_resized,
                        (x1, y1),
                        (x2, y2),
                        color,
                        2
                    )

                    cv2.putText(
                        frame_resized,
                        f"{label} {distance:.1f}cm",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        color,
                        2
                    )

    # =============================================
    # SEND TCP COMMAND
    # =============================================

    if current_label != "":

        now = time.time()

        if now - last_send_time > SEND_INTERVAL:

            # -------- STOP SIGN --------
            if current_label == "stopSign":

                if current_distance < 200:

                    last_send_time = now

                    print(f"📤 SEND: stopSign | distance: {current_distance:.1f} cm")

                    if sock:
                        try:
                            time.sleep(2)
                            sock.send(("stopSign\n").encode())
                        except:
                            print("❌ TCP send error")

            # -------- OTHER SIGNS --------
            else:

                last_send_time = now

                print("📤 SEND:", current_label)

                if sock:
                    try:
                        sock.send((current_label + "\n").encode())
                    except:
                        print("❌ TCP send error")

    # =============================================
    # DISPLAY
    # =============================================

    cv2.line(
        frame_resized,
        (0, split_line_y),
        (new_w, split_line_y),
        (255, 0, 0),
        2
    )

    cv2.imshow("YOLO Detection", frame_resized)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# =====================================================
# CLEAN UP
# =====================================================

cap.release()

if sock:
    sock.close()

cv2.destroyAllWindows()