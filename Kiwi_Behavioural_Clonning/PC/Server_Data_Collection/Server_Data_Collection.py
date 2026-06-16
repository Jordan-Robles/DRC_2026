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
OUTPUT_DIR = r'C:\Users\jorda\DRC_2026\DRC_2026\Testing_Data\Test8'
CSV_FILE = os.path.join(OUTPUT_DIR, 'labels.csv')
os.makedirs(OUTPUT_DIR, exist_ok=True)
#NUM_IMAGES = 100  #Amount of images to capture
#CAPTURE_DELAY = 0.1  #Delay between frames

# === Socket setup ===
PORT = 9999

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', PORT))
s.listen(1)
print(f"Server started. Waiting for Raspberry Pi to connect on port {PORT}...")
conn, addr = s.accept()
print(f"Connected by {addr}")

data = b''
payload_size = struct.calcsize("!I")

# === CSV setup ===
csv_file = open(CSV_FILE, 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['timestamp', 'camera_id', 'image', 'mapped_value'])

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

                # -- INTEGRATED NEW CODE --
        payload = pickle.loads(frame_data)
        frames = [cv2.imdecode(f, cv2.IMREAD_COLOR) if f is not None else None for f in payload["frames"]]
        mapped_value = payload["mapped_value"]

        resized = [cv2.resize(f, (320, 240)) if f is not None else np.zeros((240, 320, 3), np.uint8) for f in frames]
        combined = np.hstack(resized) 

        #Print to the stream the state of data collection via actuation of CH3
        capturing = payload.get("capturing", False)

        if capturing:
            cv2.putText(combined, "CAPTURING", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(combined, "PAUSED", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        # Display
        cv2.imshow('Camera Stream', combined)

        
        if capturing:
            # Save images + CSV
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')

            # Determine naming based on how many valid frames we received
            valid_frames = [f for f in frames if f is not None]
            num_cams = len(valid_frames)
            
            # Map index to camera position depending on camera count
            if num_cams == 1:
                labels = ["center"]
            elif num_cams >= 3:
                # Assuming index 0=center, 1=right, 2=left (or adjust to match your physical setup)
                labels = ["center", "right", "left"]
            else:
                # Fallback for 2 cameras just in case
                labels = ["cam0", "cam1"]

            for cam_id, frame in enumerate(valid_frames):
                if cam_id < len(labels):
                    cam_label = labels[cam_id]
                else:
                    cam_label = f"cam{cam_id}"

                angle = mapped_value 
                img_filename = f"{cam_label}_{timestamp}.jpg"
                img_path = os.path.join(OUTPUT_DIR, img_filename)
                
                # Check that the frame is actually valid before saving
                if frame is not None:
                    cv2.imwrite(img_path, frame)
                    csv_writer.writerow([timestamp, cam_label, img_filename, angle])
                    print(f"[{frame_counter}] Saved {img_filename} with angle {angle}")

            csv_file.flush()

            frame_counter += 1

        # Press ESC to stop
        if cv2.waitKey(1) & 0xFF == 27:
            print("ESC pressed, stopping capture.")
            break

        #time.sleep(CAPTURE_DELAY)

    except Exception as e:
        print(f"Error: {e}")
        break
    except KeyboardInterrupt:
        print("stopped manually")
        break

# Cleanup
csv_file.close()
cv2.destroyAllWindows()
conn.close()
s.close()
