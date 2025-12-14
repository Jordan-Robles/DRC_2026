import socket
import cv2
import pickle
import struct
import numpy as np
import csv
import os
import time
from datetime import datetime
import random

# === CONFIG ===
OUTPUT_DIR = r'C:\Users\jorda\DRC\Testing_Data\Test3'
CSV_FILE = os.path.join(OUTPUT_DIR, 'labels.csv')
os.makedirs(OUTPUT_DIR, exist_ok=True)
#NUM_IMAGES = 100  #Amount of images to capture
CAPTURE_DELAY = 0.1  #Delay between frames

# === Socket setup ===
HOST = '0.0.0.0'
PORT = 9999

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn, addr = s.accept()
print(f"Connected by {addr}")

data = b''
payload_size = struct.calcsize("!I")

# === CSV setup ===
csv_file = open(CSV_FILE, 'w', newline='')
csv_writer = csv.writer(csv_file)
#csv_writer.writerow(['timestamp', 'camera_id', 'image', 'steering_angle'])

frame_counter = 0

while True:
    try:
        # Get size prefix
        while len(data) < payload_size:
            packet = conn.recv(4096)
            if not packet:
                raise ConnectionError("Socket closed")
            data += packet

        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("!I", packed_msg_size)[0]

        # Receive frame data
        while len(data) < msg_size:
            packet = conn.recv(4096)
            if not packet:
                raise ConnectionError("Socket closed")
            data += packet

        frame_data = data[:msg_size]
        data = data[msg_size:]

        frames = pickle.loads(frame_data)
        resized = [cv2.resize(f, (320, 240)) if f is not None else np.zeros((240, 320, 3), np.uint8) for f in frames]
        combined = np.hstack(resized) 

        # Display
        cv2.imshow('Camera Stream', combined)

      # Save images + CSV
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')

        for cam_id, frame in enumerate(resized):
            angle = round(random.uniform(-1, 1), 2)
            img_filename = f"centre_{timestamp}.jpg" #cam{cam_id}_{timestamp}.jpg
            img_path = os.path.join(OUTPUT_DIR, img_filename)
            cv2.imwrite(img_path, frame)
            csv_writer.writerow([timestamp, cam_id, img_filename, angle])
            print(f"[{frame_counter}] Saved {img_filename} with angle {angle}")

        frame_counter += 1

        # Press ESC to stop
        if cv2.waitKey(1) & 0xFF == 27:
            print("ESC pressed, stopping capture.")
            break

        time.sleep(CAPTURE_DELAY)

    except Exception as e:
        print(f"Error: {e}")
        break

# Cleanup
csv_file.close()
cv2.destroyAllWindows()
conn.close()
s.close()
