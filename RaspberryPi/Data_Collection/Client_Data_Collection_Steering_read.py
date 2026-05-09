import socket
import cv2
import pickle
import struct
import serial
import threading

# === Config ===
SERVER_IP = '192.168.0.225'
PORT = 9999
cam_indexes = [0]
SERIAL_PORT = '/dev/ttyUSB0'  # Change to /dev/ttyACM0 if needed
BAUD_RATE = 9600

# === Serial Setup ===
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Thread-safe store for the latest RC value
latest_value = 0.0
serial_lock = threading.Lock()

def serial_reader():
    """Background thread: reads mapped_value lines from Arduino."""
    global latest_value
    buffer = ""
    while True:
        try:
            char = ser.read(1).decode('utf-8', errors='ignore')
            if char == '\n':
                line = buffer.strip()
                buffer = ""
                if line:
                    try:
                        val = float(line)
                        with serial_lock:
                            latest_value = val
                    except ValueError:
                        pass  # Ignore non-float lines (e.g. "RC Ready")
            else:
                buffer += char
        except Exception as e:
            print(f"Serial read error: {e}")
            break

# Start background serial reader thread
reader_thread = threading.Thread(target=serial_reader, daemon=True)
reader_thread.start()

# === Camera Setup ===
cams = [cv2.VideoCapture(i) for i in cam_indexes]
for cam in cams:
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# === Connect to Laptop ===
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((SERVER_IP, PORT))
print("Connected to server")

try:
    while True:
        frames = []
        for cam in cams:
            ret, frame = cam.read()
            if not ret:
                frame = None
            frames.append(frame)

        # === Grab latest RC value from Arduino ===
        with serial_lock:
            mapped_value = latest_value

        # === Package frames + mapped value ===
        payload = {
            "frames": frames,
            "mapped_value": mapped_value  # -1.0 to 1.0 from Arduino
        }

        data = pickle.dumps(payload)
        message = struct.pack("!I", len(data)) + data
        client_socket.sendall(message)

except KeyboardInterrupt:
    print("Interrupted, closing...")

finally:
    for cam in cams:
        cam.release()
    client_socket.close()
    ser.close()