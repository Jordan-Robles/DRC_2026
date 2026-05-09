"""Kiwi_Control controller - RC Steering Test"""
from controller import Robot
import cv2
import numpy as np
import serial

# ── CONFIG ────────────────────────────────────────────────────────────────────
SERIAL_PORT      = 'COM4'
BAUD_RATE        = 9600
SERIAL_TIMEOUT   = 0.02
PWM_MIN          = 1000
PWM_CENTER       = 1500
PWM_MAX          = 2000
FORWARD_SPEED    = 20.0
MAX_REAR_SPEED   = 25.0
TURN_SPEED_FACTOR = 0.4  # fraction of FORWARD_SPEED at full steering lock
# ─────────────────────────────────────────────────────────────────────────────

def pwm_to_steering(pwm_us):
    pwm_us = max(PWM_MIN, min(PWM_MAX, pwm_us))
    if pwm_us >= PWM_CENTER:
        return (pwm_us - PWM_CENTER) / (PWM_MAX - PWM_CENTER)
    else:
        return -((PWM_CENTER - pwm_us) / (PWM_CENTER - PWM_MIN))

def open_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=SERIAL_TIMEOUT)
        ser.reset_input_buffer()
        print(f"[Serial] Connected -> {port} @ {baud} baud")
        return ser
    except serial.SerialException as e:
        print(f"[Serial] WARN - {e}  |  steering defaults to 0.0")
        return None

def read_steering(ser, last_val):
    if ser is None:
        return 0.0
    try:
        latest = last_val
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.lstrip('-').isdigit():
                latest = pwm_to_steering(int(line))
        return latest
    except Exception:
        pass
    return last_val

if __name__ == "__main__":
    kiwi      = Robot()
    timestep  = 64

    # ── Devices ──
    camera = kiwi.getDevice('camera')
    camera.enable(timestep)
    width  = camera.getWidth()
    height = camera.getHeight()

    right_motor = kiwi.getDevice('Right_motor')
    left_motor  = kiwi.getDevice('Left_motor')
    rear_motor  = kiwi.getDevice('Rear_motor')

    # ── Null check ──
    for name, motor in [('Right_motor', right_motor), ('Left_motor', left_motor), ('Rear_motor', rear_motor)]:
        if motor is None:
            print(f"[ERROR] Device '{name}' not found - check spelling in .wbt")
        else:
            print(f"[OK] Found {name}")

    for motor in (right_motor, left_motor, rear_motor):
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)

    # ── Motor debug ──
    print(f"[Motors] right={right_motor}  left={left_motor}  rear={rear_motor}")
    print(f"[Motors] max velocity  right={right_motor.getMaxVelocity():.2f}  left={left_motor.getMaxVelocity():.2f}  rear={rear_motor.getMaxVelocity():.2f}")

    # ── Serial ──
    ser      = open_serial(SERIAL_PORT, BAUD_RATE)
    steering = 0.0

    # ── Main loop ──
    while kiwi.step(timestep) != -1:

        # Camera
        image = camera.getImage()
        img = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
        img = img[:, :, :3]
        
        img = img[60:, :]
        cv2.imshow("camera", img)
        cv2.waitKey(1)

        # Steering from RC
        steering = read_steering(ser, steering)

        # Scale drive speed down when steering to prevent rear wheel lockup
        speed_scale = 1.0 - (1.0 - TURN_SPEED_FACTOR) * abs(steering)
        drive_cmd   = -FORWARD_SPEED * speed_scale
        rear_cmd    =  steering * min(MAX_REAR_SPEED, rear_motor.getMaxVelocity())

        print(f"steering={steering:+.4f}  drive={drive_cmd:+.3f}  rear={rear_cmd:+.3f}")

        # Drive
        right_motor.setVelocity(drive_cmd)
        left_motor.setVelocity(drive_cmd)
        rear_motor.setVelocity(rear_cmd)

    cv2.destroyAllWindows()
    if ser:
        ser.close()