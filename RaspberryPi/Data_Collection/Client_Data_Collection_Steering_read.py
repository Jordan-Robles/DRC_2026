import socket
import cv2
import pickle
import struct
import pigpio

# === Config ===
SERVER_IP = '192.168.0.225'
PORT = 9999
cam_indexes = [0]
RC_PIN = 4  # GPIO pin connected to RC receiver signal (adjust as needed)

# === PWM Reading Setup ===
pi = pigpio.pi()
if not pi.connected:
    raise Exception("Failed to connect to pigpio daemon")

# Variables to store pulse width
pulse_width = 1500  # Default middle position

def pwm_callback(gpio, level, tick):
    global pulse_width, pulse_start
    if level == 1:  # Rising edge
        pulse_start = tick
    elif level == 0:  # Falling edge
        pulse_width = tick - pulse_start

# Set up callback for PWM reading
pulse_start = 0
pi.set_mode(RC_PIN, pigpio.INPUT)
pi.callback(RC_PIN, pigpio.EITHER_EDGE, pwm_callback)

def map_pwm_to_range(pwm_value, min_pwm=1000, max_pwm=2000, min_out=-1.0, max_out=1.0):
    """Map PWM value (typically 1000-2000) to desired range (-1 to 1)"""
    # Clamp to expected range
    pwm_value = max(min_pwm, min(max_pwm, pwm_value))
    # Map to output range
    return min_out + (pwm_value - min_pwm) * (max_out - min_out) / (max_pwm - min_pwm)

# === Camera setup ===
cams = [cv2.VideoCapture(i) for i in cam_indexes]
for cam in cams:
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640) #PS3_resolution
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# === Connect to laptop ===
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

        # === Read current PWM pulse width and map to -1 to 1 ===
        # RC receivers typically output 1000-2000 microseconds
        # 1500 is center/neutral position (0.0)
        current_pulse = pulse_width
        mapped_value = map_pwm_to_range(current_pulse)

        # === Package frames + mapped value ===
        payload = {
            "frames": frames,
            "mapped_value": mapped_value  # Mapped to -1.0 to 1.0
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
    pi.stop()  # Cleanup pigpio connection
