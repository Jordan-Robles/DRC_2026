import cv2
import numpy as np
import math
import time

# ----------------------------
# CONFIG (LOW RES = STABLE)
# ----------------------------
WIDTH = 320
HEIGHT = 240
FPS = 30

CAMERAS = [
    "/dev/video0",
    "/dev/video1",
    "/dev/video2",
    "/dev/video3",
    "/dev/video4",
    "/dev/video5",
    "/dev/video6",
    "/dev/video7"
]

# ----------------------------
# OPEN CAMERA (robust)
# ----------------------------
def open_camera(path):
    print(f"Opening {path} ...")

    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)

    # Force stable format
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # 🔥 IMPORTANT: reduce bandwidth
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    # reduce buffering latency
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    time.sleep(0.5)  # let driver settle

    if not cap.isOpened():
        print(f"❌ FAILED to open {path}")
        return None

    # warm-up frames (VERY IMPORTANT for UVC cameras)
    for _ in range(10):
        cap.read()

    print(f"✅ OPENED {path}")
    return cap


# ----------------------------
# OPEN ALL CAMERAS (staggered)
# ----------------------------
caps = []

for cam in CAMERAS:
    cap = open_camera(cam)
    caps.append(cap)

    # 🔥 STAGGER STARTUP (key fix)
    time.sleep(1.0)


# filter out failed cameras
valid_caps = []
valid_names = []

for i, cap in enumerate(caps):
    if cap is not None:
        valid_caps.append(cap)
        valid_names.append(CAMERAS[i])

print(f"\nActive cameras: {valid_names}")

if len(valid_caps) == 0:
    print("No working cameras.")
    exit()


# ----------------------------
# GRID SETUP
# ----------------------------
n = len(valid_caps)
cols = math.ceil(math.sqrt(n))
rows = math.ceil(n / cols)


# ----------------------------
# MAIN LOOP
# ----------------------------
while True:
    frames = []

    for i, cap in enumerate(valid_caps):
        ret, frame = cap.read()

        if not ret or frame is None:
            frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
            cv2.putText(frame, "NO SIGNAL",
                        (40, 120),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 0, 255), 2)
        else:
            frame = cv2.resize(frame, (WIDTH, HEIGHT))
            cv2.putText(frame,
                        valid_names[i],
                        (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1)

        frames.append(frame)

    # pad grid if needed
    while len(frames) < rows * cols:
        frames.append(np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8))

    # build grid
    grid_rows = []
    for r in range(rows):
        row = cv2.hconcat(frames[r * cols:(r + 1) * cols])
        grid_rows.append(row)

    grid = cv2.vconcat(grid_rows)

    cv2.imshow("Multi Camera Feed (Stable)", grid)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# ----------------------------
# CLEAN EXIT
# ----------------------------
for cap in valid_caps:
    cap.release()

cv2.destroyAllWindows()
