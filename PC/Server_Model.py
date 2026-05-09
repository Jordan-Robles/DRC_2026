import socket
import cv2
import pickle
import struct
import numpy as np
from keras.models import load_model
from keras.models import load_model
from keras.losses import MeanSquaredError
from keras.optimizers import Adam
from Colour_filters import ColourFilter



# === CONFIG ===
PORT = 9999  
MODEL_PATH = r'C:\Users\jorda\DRC_2026\DRC_2026\Trained_Models\model_DRC_2026Test_imgPro.h5'#r'C:\Users\jorda\DRC\DRC_Code1\Course\model.h5' #r'C:\Users\jorda\DRC\DRC_Code1\model_DRC_V2.h5' 
# === Load the model ===
print("Loading model...")
model = load_model(MODEL_PATH, compile=False)  # Do not auto-load 'mse'
print("Model loaded.")

# === Recompile model with the same loss & optimizer ===
model.compile(loss=MeanSquaredError(), optimizer=Adam(learning_rate=1e-3)) #1e-4
print("Model compiled successfully.")


# === Setup server socket ===
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('', PORT))
server_socket.listen(1)
print(f"Server listening on port {PORT}...")

conn, addr = server_socket.accept()
print(f"Connected by {addr}")

data = b""
payload_size = struct.calcsize("!I")  # Use 'Q' if using 64-bit systems

try:
    while True:
        # Receive frame size
        while len(data) < payload_size:
            packet = conn.recv(4096)
            if not packet:
                raise ConnectionError("Client disconnected")
            data += packet

        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("!I", packed_msg_size)[0]

        # Receive image frame
        while len(data) < msg_size:
            data += conn.recv(4096)

        frame_data = data[:msg_size]
        data = data[msg_size:]

        # Decode image
        frame = pickle.loads(frame_data)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        #image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        if frame is None:
            continue

        # Show live stream
        cv2.imshow("Live Stream", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC key
            print("ESC pressed. Exiting...")
            break
        
        # Predict steering angle
        input_image = ColourFilter.img_preprocess(frame)
        input_image = np.expand_dims(input_image, axis=0)
        steering_angle = float(model.predict(input_image)[0][0])
        print(f"Predicted Steering: {steering_angle:.2f}")

        # Send angle back to Raspberry Pi
        angle_bytes = f"{steering_angle:.2f}".encode('utf-8')
        conn.sendall(angle_bytes)

except KeyboardInterrupt:
    print("KeyboardInterrupt received. Exiting...")

except Exception as e:
    print(f"Error: {e}")

finally:
    cv2.destroyAllWindows()
    conn.close()
    server_socket.close()
    print("Resources cleaned up. Server shut down.")
