import socket
import cv2
import pickle
import struct
import serial
import time
import sys

# === CONFIG ===
LAPTOP_IP = '172.20.10.2'  # Replace with your laptop IP
PORT = 9999
SERIAL_PORT = '/dev/ttyUSB0'  # or '/dev/ttyACM0'
BAUDRATE = 9600

# === Serial setup ===
try:
    arduino = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2)  # Wait for Arduino to initialize
    print(f"Connected to Arduino on {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Could not open serial port {SERIAL_PORT}: {e}")
    sys.exit(1)

# === Socket setup ===
try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((LAPTOP_IP, PORT))
    print(f"Connected to laptop at {LAPTOP_IP}:{PORT}")
except socket.error as e:
    print(f"Socket connection error: {e}")
    arduino.close()
    sys.exit(1)

# === Camera setup ===
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")
    arduino.close()
    client_socket.close()
    sys.exit(1)

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Warning: Frame not captured.")
            continue

        # Encode image
        result, frame_encoded = cv2.imencode('.jpg', frame, encode_param)
        data = pickle.dumps(frame_encoded)
        message_size = struct.pack("L", len(data))  # 'L' = unsigned long

        try:
            # Send frame to laptop
            client_socket.sendall(message_size + data)

            # Receive steering angle from laptop
            angle_bytes = client_socket.recv(1024)
            angle_str = angle_bytes.decode('utf-8').strip()
            print(f"Received angle: {angle_str}")

            # Forward to Arduino
            arduino.write((angle_str + "\n").encode('utf-8'))

        except BrokenPipeError:
            print("Error: Broken pipe. Laptop server probably disconnected.")
            break
        except ConnectionResetError:
            print("Error: Connection reset by peer.")
            break
        except Exception as e:
            print(f"Unexpected error during socket or serial communication: {e}")
            break

except KeyboardInterrupt:
    print("Interrupted by user. Exiting...")

finally:
    print("Cleaning up...")
    cap.release()
    arduino.close()
    client_socket.close()
    print("Shutdown complete.")
