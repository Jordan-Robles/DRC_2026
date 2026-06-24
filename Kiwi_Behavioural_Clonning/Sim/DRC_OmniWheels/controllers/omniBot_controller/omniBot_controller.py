"""omni_robot_test controller - RC Steering, Kiwi/Omni Drive"""
from controller import Robot, InertialUnit
import cv2
import numpy as np
import serial

# ── CONFIG ────────────────────────────────────────────────────────────────────
SERIAL_PORT       = 'COM4'
BAUD_RATE         = 9600
SERIAL_TIMEOUT    = 0.02

PWM_MIN           = 1000
PWM_CENTER        = 1500
PWM_MAX           = 2000

# Kiwi drive params
MAX_SPEED         = 40.0   # rad/s (matches PROTO maxVelocity)
FORWARD_SPEED     = 15.0
STRAFE_SPEED      = 15.0
TURN_SPEED        = 10.0
TURN_SPEED_FACTOR = 0.4    # speed reduction fraction at full steering lock

# Wheel layout (angles from PROTO rotations: 0, 120, 240 deg)
# wheel1 = SOLID3 rotation 4.18 rad (~240°)
# wheel2 = SOLID1 rotation 0 rad (0°)
# wheel3 = SOLID2 rotation 2.09 rad (~120°)
WHEEL_ANGLES_DEG = [240.0, 0.0, 120.0]   # wheel1, wheel2, wheel3
# ─────────────────────────────────────────────────────────────────────────────

def pwm_to_steering(pwm_us):
    pwm_us = max(PWM_MIN, min(PWM_MAX, pwm_us))
    if pwm_us >= PWM_CENTER:
        return  (pwm_us - PWM_CENTER) / (PWM_MAX - PWM_CENTER)
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

def kiwi_ik(vx, vy, omega, wheel_angles_deg):
    """
    Inverse kinematics for 3-omniwheel (kiwi) drive.
    vx, vy  : desired body-frame velocity (forward/strafe)
    omega   : desired yaw rate (rad/s)
    Returns list of wheel velocities [w1, w2, w3].
    
    For each wheel i at angle theta_i:
      w_i = -sin(theta_i)*vx + cos(theta_i)*vy + omega
    The sign convention here drives the robot forward along +X.
    """
    speeds = []
    for deg in wheel_angles_deg:
        rad = np.radians(deg)
        w = -np.sin(rad) * vx + np.cos(rad) * vy + omega
        speeds.append(w)
    return speeds

def clamp_speeds(speeds, max_speed):
    """Scale all wheels down proportionally if any exceeds max_speed."""
    peak = max(abs(s) for s in speeds)
    if peak > max_speed:
        scale = max_speed / peak
        speeds = [s * scale for s in speeds]
    return speeds


if __name__ == "__main__":
    robot    = Robot()
    timestep = 64

    # ── Devices ──────────────────────────────────────────────────────────────
    camera = robot.getDevice('camera')
    camera.enable(timestep)
    width  = camera.getWidth()
    height = camera.getHeight()

    imu = robot.getDevice('IMU')
    imu.enable(timestep)

    wheel1 = robot.getDevice('wheel1')
    wheel2 = robot.getDevice('wheel2')
    wheel3 = robot.getDevice('wheel3')

    enc1 = robot.getDevice('pw1')
    enc2 = robot.getDevice('pw2')
    enc3 = robot.getDevice('pw3')

    # ── Null checks ──────────────────────────────────────────────────────────
    for name, dev in [('camera', camera), ('IMU', imu),
                      ('wheel1', wheel1), ('wheel2', wheel2), ('wheel3', wheel3),
                      ('pw1', enc1),     ('pw2', enc2),      ('pw3', enc3)]:
        if dev is None:
            print(f"[ERROR] Device '{name}' not found - check spelling in .wbt")
        else:
            print(f"[OK] Found {name}")

    # ── Motor init ───────────────────────────────────────────────────────────
    for motor in (wheel1, wheel2, wheel3):
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)

    for enc in (enc1, enc2, enc3):
        enc.enable(timestep)

    print(f"[Motors] max vel  w1={wheel1.getMaxVelocity():.1f}"
          f"  w2={wheel2.getMaxVelocity():.1f}"
          f"  w3={wheel3.getMaxVelocity():.1f}")

    # ── Serial ───────────────────────────────────────────────────────────────
    ser      = open_serial(SERIAL_PORT, BAUD_RATE)
    steering = 0.0

    # ── Main loop ─────────────────────────────────────────────────────────────
    while robot.step(timestep) != -1:

        # -- Camera -----------------------------------------------------------
        image = camera.getImage()
        img = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
        img = img[:, :, :3]
        cv2.imshow("camera", img)
        cv2.waitKey(1)

        # -- IMU --------------------------------------------------------------
        roll, pitch, yaw = imu.getRollPitchYaw()

        # -- Encoders ---------------------------------------------------------
        e1 = enc1.getValue()
        e2 = enc2.getValue()
        e3 = enc3.getValue()

        # -- RC steering ------------------------------------------------------
        steering = read_steering(ser, steering)

        # -- Drive mapping ----------------------------------------------------
        # steering [-1, +1] maps to:
        #   lateral translation left/right (vy)
        #   yaw component (omega) scaled by TURN_SPEED_FACTOR
        speed_scale = 1.0 - (1.0 - TURN_SPEED_FACTOR) * abs(steering)

        vx    =  FORWARD_SPEED * speed_scale   # always drive forward
        vy    =  STRAFE_SPEED  * steering       # strafe with steering
        omega = -TURN_SPEED    * steering       # counter-rotate for arc

        # -- IK + clamp -------------------------------------------------------
        speeds = kiwi_ik(vx, vy, omega, WHEEL_ANGLES_DEG)
        speeds = clamp_speeds(speeds, MAX_SPEED)
        w1, w2, w3 = speeds

        print(f"steer={steering:+.3f}  vx={vx:+.2f}  vy={vy:+.2f}  omega={omega:+.2f}"
              f"  |  w1={w1:+.2f}  w2={w2:+.2f}  w3={w3:+.2f}"
              f"  |  yaw={np.degrees(yaw):+.1f}°"
              f"  |  enc=[{e1:.2f}, {e2:.2f}, {e3:.2f}]")

        wheel1.setVelocity(w1)
        wheel2.setVelocity(w2)
        wheel3.setVelocity(w3)

    cv2.destroyAllWindows()
    if ser:
        ser.close()