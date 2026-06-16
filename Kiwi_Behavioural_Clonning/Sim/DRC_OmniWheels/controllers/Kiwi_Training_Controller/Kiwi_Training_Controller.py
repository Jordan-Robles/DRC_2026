"""
Webots Behaviour Cloning Data Collector  –  Kiwi Drive (3-wheel omni)
=======================================================================
Place this file in your Webots controller folder (replace / alongside
your existing controller).

RC controller → Arduino Uno → Serial → this script
Camera images + steering saved to CSV identical to your real-robot format:
    [timestamp, camera_id, image_filename, steering_angle]

Kiwi-drive kinematics (same as your test controller):
    v_rear  =  x + w
    v_left  = -0.5*x + 0.866*y + w
    v_right = -0.5*x - 0.866*y + w

During data collection we keep  x=0, y=FORWARD_SPEED
and map RC steering → w  (rotation component).
"""

from controller import Robot
import csv
import os
import time
import serial
from datetime import datetime
import numpy as np
import cv2

# ── USER CONFIG ────────────────────────────────────────────────────────────────
OUTPUT_DIR       = r'C:\Users\jorda\Desktop\Code\Python\DRC\DRC_2026\Virutal_Data\Data1'   # ← change me
CSV_FILENAME     = 'labels.csv'

SERIAL_PORT      = 'COM3'      # ← your Arduino COM port  (Linux: /dev/ttyUSB0)
BAUD_RATE        = 9600
SERIAL_TIMEOUT   = 0.02        # keep short – must not block Webots step

# RC PWM calibration (microseconds) – check with Arduino Serial Monitor
PWM_MIN          = 1000
PWM_CENTER       = 1500
PWM_MAX          = 2000

# Webots device names – must match your .wbt exactly
CAMERA_NAME      = 'camera'
RIGHT_MOTOR_NAME = 'Right_motor'
LEFT_MOTOR_NAME  = 'Left_motor'
REAR_MOTOR_NAME  = 'Rear_motor'

# Motion parameters
FORWARD_SPEED    = 4.0   # constant velocity for left & right motors
MAX_REAR_SPEED   = 4.0   # rear motor velocity at full steering lock

# Save one frame every N simulation steps  (64 ms step → every 3 steps ≈ 5 fps)
SAVE_EVERY_N     = 3

# Show live preview window (set False if running headless)
SHOW_PREVIEW     = True
# ──────────────────────────────────────────────────────────────────────────────


def pwm_to_steering(pwm_us: int) -> float:
    """Raw PWM microseconds → normalised steering [-1.0 … +1.0]."""
    pwm_us = max(PWM_MIN, min(PWM_MAX, pwm_us))
    if pwm_us >= PWM_CENTER:
        return (pwm_us - PWM_CENTER) / (PWM_MAX - PWM_CENTER)
    else:
        return -((PWM_CENTER - pwm_us) / (PWM_CENTER - PWM_MIN))


def open_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=SERIAL_TIMEOUT)
        time.sleep(2)
        ser.reset_input_buffer()
        print(f"[Serial] Connected -> {port} @ {baud} baud")
        return ser
    except serial.SerialException as e:
        print(f"[Serial] WARN - {e}")
        print("         Steering defaults to 0.0. Connect Arduino and restart.")
        return None


def read_steering(ser, last_val):
    """Non-blocking serial read; returns last known value if nothing new."""
    if ser is None:
        return 0.0
    try:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.lstrip('-').isdigit():
                return pwm_to_steering(int(line))
    except Exception:
        pass
    return last_val


# ── MAIN ───────────────────────────────────────────────────────────────────────
if __name__ == '__main__':

    kiwi     = Robot()
    timestep = 64   # ms – matches your test controller

    # ── Devices ──
    camera = kiwi.getDevice(CAMERA_NAME)
    camera.enable(timestep)
    CAM_W  = camera.getWidth()
    CAM_H  = camera.getHeight()
    print(f"[Camera] {CAM_W}x{CAM_H} px  |  timestep {timestep} ms")

    right_motor = kiwi.getMotor(RIGHT_MOTOR_NAME)
    left_motor  = kiwi.getMotor(LEFT_MOTOR_NAME)
    rear_motor  = kiwi.getMotor(REAR_MOTOR_NAME)

    for motor in (right_motor, left_motor, rear_motor):
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)

    # ── Serial ──
    ser = open_serial(SERIAL_PORT, BAUD_RATE)

    # ── Output ──
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    csv_path = os.path.join(OUTPUT_DIR, CSV_FILENAME)
    csv_file   = open(csv_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    # Uncomment to add header row:
    # csv_writer.writerow(['timestamp', 'camera_id', 'image', 'steering_angle'])

    print(f"[Output] -> {csv_path}")
    print("[Ready]  Steer your RC transmitter to collect data.\n")

    step_count    = 0
    frame_count   = 0
    steering      = 0.0

    try:
        while kiwi.step(timestep) != -1:
            step_count += 1

            # ── Read steering from Arduino ──
            steering = read_steering(ser, steering)

            # ── Drive robot ──
            # Left & right motors drive forward at constant speed
            # Rear motor steers: RC input maps directly to its velocity
            right_motor.setVelocity(-FORWARD_SPEED)
            left_motor.setVelocity(-FORWARD_SPEED)
            rear_motor.setVelocity(-steering * MAX_REAR_SPEED)

            # ── Grab & display camera ──
            raw = camera.getImage()
            if raw is None:
                continue

            img = np.frombuffer(raw, dtype=np.uint8).reshape((CAM_H, CAM_W, 4))
            img = img[:, :, :3]   # drop alpha (BGRA -> BGR)

            if SHOW_PREVIEW:
                cv2.imshow("Webots Camera", img)
                cv2.waitKey(1)

            # ── Save at reduced rate ──
            if step_count % SAVE_EVERY_N != 0:
                continue

            img_resized  = cv2.resize(img, (320, 240))
            timestamp    = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            img_filename = f"centre_{timestamp}.jpg"
            img_path     = os.path.join(OUTPUT_DIR, img_filename)

            cv2.imwrite(img_path, img_resized)

            steering_val = round(steering, 4)
            csv_writer.writerow([timestamp, 0, img_filename, steering_val])
            csv_file.flush()

            frame_count += 1
            if frame_count % 20 == 0:
                print(f"[{frame_count:>5} frames]  steering={steering_val:+.4f}  "
                      f"rear={steering_val * MAX_REAR_SPEED:+.3f} rad/s")

    except KeyboardInterrupt:
        print("\n[Stopped]")

    finally:
        csv_file.close()
        if ser:
            ser.close()
        cv2.destroyAllWindows()
        print(f"[Done] {frame_count} frames saved -> {csv_path}")