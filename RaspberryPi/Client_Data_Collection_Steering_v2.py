import socket
import cv2
import pickle
import struct
import serial
import time
from collections import deque

# === CONFIG ===
SERVER_IP = '192.168.0.225' #'172.20.10.2'  # <-- Your laptop IP
PORT = 9999
CAM_INDEXES = [0]

# === Initialize Arduino Serial (CHANGED) ===
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Ensure this matches your port

# === Optional: Buffer to Smooth Serial Input (NEW) ===
value_buffer = deque(maxlen=5)  # Store last 5 readings for smoothing

# === Serial Reading Function (NEW) ===
def read_mapped_value():
    """Read one valid int line from Arduino Serial."""
    try:
        if arduino.in_waiting > 0:
            line = arduino.readline().decode('utf-8').strip()
            if line.isdigit():
                return int(line)
    except Exception as e:
        print(f"[WARNING] Serial read failed: {e}")
    return None

# === Optional: Smoothing Function (NEW) ===
def get_smoothed_value():
    value = read_mapped_value()
    if value is not None:
        value_buffer.append(value)
    if value_buffer:
        return int(sum(value_buffer) / len(value_buffer))
    return None

# === Camera Setup ===
cams = [cv2.VideoCapture(i) for i in CAM_INDEXES]
for cam in cams:
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# === Connect to Laptop ===
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((SERVER_IP, PORT))

try:
    while True:
        frames = []
        for cam in cams:
            ret, frame = cam.read()
            if not ret:
                frame = None
            frames.append(frame)

        # === Read Arduino Value (CHANGED) ===
        mapped_value = get_smoothed_value()  # <- use smoothed reading
        # mapped_value = read_mapped_value()  # <- OR use raw reading (less stable)

        # === Pack frames and value ===
        payload = {
            "frames": frames,
            "mapped_value": mapped_value
        }

        data = pickle.dumps(payload)
        message = struct.pack("!I", len(data)) + data
        client_socket.sendall(message)

        time.sleep(0.05)  # optional: throttle loop

except KeyboardInterrupt:
    print("Interrupted by user, closing...")

finally:
    for cam in cams:
        cam.release()
    client_socket.close()
    arduino.close()
