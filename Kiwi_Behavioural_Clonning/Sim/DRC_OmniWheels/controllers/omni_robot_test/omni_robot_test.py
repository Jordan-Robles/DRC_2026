"""omni_robot_test controller - Hardware/Motion Test Sequence"""
import sys
import traceback

try:
    from controller import Robot
    import cv2
    import numpy as np
except ImportError as e:
    print(f"[IMPORT ERROR] {e}", flush=True)
    sys.exit(1)

print("[OK] Imports successful", flush=True)

# ── CONFIG ────────────────────────────────────────────────────────────────────
MAX_SPEED    = 40.0
TEST_SPEED   = 12.0
TURN_SPEED   = 8.0
TIMESTEP     = 64

WHEEL_ANGLES_DEG = [240.0, 0.0, 120.0]  # wheel1, wheel2, wheel3

# Test sequence: (label, vx, vy, omega, duration_ms)
TEST_SEQUENCE = [
    ("STATIONARY",   0.0,  0.0,  0.0,  2000),
    ("FORWARD",     15.0,  0.0,  0.0,  2000),
    ("STOP",         0.0,  0.0,  0.0,  1000),
    ("TURN RIGHT",   0.0,  0.0, -8.0,  2000),
    ("STOP",         0.0,  0.0,  0.0,  1000),
    ("TURN LEFT",    0.0,  0.0,  8.0,  2000),
    ("STOP",         0.0,  0.0,  0.0,  1000),
    ("STRAFE RIGHT",  0.0, -12.0, 0.0, 2000),
    ("STOP",         0.0,  0.0,  0.0,  1000),
    ("STRAFE LEFT",  0.0, 12.0,  0.0,  2000),
    ("STOP",         0.0,  0.0,  0.0,  2000),
    ("DONE",         0.0,  0.0,  0.0,  0),
]
# ─────────────────────────────────────────────────────────────────────────────

def kiwi_ik(vx, vy, omega, wheel_angles_deg):
    speeds = []
    for deg in wheel_angles_deg:
        rad = np.radians(deg)
        w = -np.sin(rad) * vx + np.cos(rad) * vy + omega
        speeds.append(w)
    return speeds

def clamp_speeds(speeds, max_speed):
    peak = max(abs(s) for s in speeds)
    if peak > max_speed:
        scale = max_speed / peak
        speeds = [s * scale for s in speeds]
    return speeds

def draw_overlay(img, label, step_idx, total_steps, vx, vy, omega, w1, w2, w3, yaw_deg, encoders):
    overlay = img.copy()
    h, w = overlay.shape[:2]

    # Semi-transparent black bar at top
    cv2.rectangle(overlay, (0, 0), (w, 90), (0, 0, 0), -1)
    img = cv2.addWeighted(overlay, 0.55, img, 0.45, 0)

    # Step label + progress
    cv2.putText(img, f"STEP {step_idx}/{total_steps}: {label}",
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 100), 2)

    # vx / vy / omega
    cv2.putText(img, f"vx={vx:+.1f}  vy={vy:+.1f}  omega={omega:+.1f}",
                (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 220, 0), 1)

    # Wheel speeds
    cv2.putText(img, f"w1={w1:+.2f}  w2={w2:+.2f}  w3={w3:+.2f}",
                (10, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (100, 200, 255), 1)

    # Yaw + encoders
    cv2.putText(img, f"yaw={yaw_deg:+.1f}deg   enc=[{encoders[0]:.1f}, {encoders[1]:.1f}, {encoders[2]:.1f}]",
                (10, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    # Direction arrow (bottom right)
    cx, cy = w - 60, h - 60
    arrow_len = 35
    if abs(vx) > 0.1 or abs(vy) > 0.1:
        ax = int(cx + vy / max(abs(vx), abs(vy), 1) * arrow_len)
        ay = int(cy - vx / max(abs(vx), abs(vy), 1) * arrow_len)
        cv2.arrowedLine(img, (cx, cy), (ax, ay), (0, 255, 100), 2, tipLength=0.3)
    if abs(omega) > 0.1:
        color = (0, 100, 255) if omega > 0 else (0, 200, 255)
        cv2.putText(img, "CCW" if omega > 0 else "CW",
                    (cx - 15, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    return img


if __name__ == "__main__":
    robot    = Robot()

    # ── Devices ──────────────────────────────────────────────────────────────
    camera = robot.getDevice('camera')
    camera.enable(TIMESTEP)
    width  = camera.getWidth()
    height = camera.getHeight()

    imu = robot.getDevice('IMU')
    imu.enable(TIMESTEP)

    wheel1 = robot.getDevice('wheel1')
    wheel2 = robot.getDevice('wheel2')
    wheel3 = robot.getDevice('wheel3')

    enc1 = robot.getDevice('pw1')
    enc2 = robot.getDevice('pw2')
    enc3 = robot.getDevice('pw3')

    # ── Null checks ──────────────────────────────────────────────────────────
    all_ok = True
    for name, dev in [('camera', camera), ('IMU', imu),
                      ('wheel1', wheel1), ('wheel2', wheel2), ('wheel3', wheel3),
                      ('pw1', enc1),      ('pw2', enc2),      ('pw3', enc3)]:
        if dev is None:
            print(f"[ERROR] Device '{name}' not found")
            all_ok = False
        else:
            print(f"[OK] Found '{name}'")

    if not all_ok:
        print("[ABORT] Fix missing devices before running test.")
        exit(1)

    # ── Init ─────────────────────────────────────────────────────────────────
    for motor in (wheel1, wheel2, wheel3):
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)

    for enc in (enc1, enc2, enc3):
        enc.enable(TIMESTEP)

    print(f"\n[Motors] max vel  w1={wheel1.getMaxVelocity():.1f}"
          f"  w2={wheel2.getMaxVelocity():.1f}"
          f"  w3={wheel3.getMaxVelocity():.1f}\n")

    # ── Test sequence state ───────────────────────────────────────────────────
    step_idx      = 0
    step_elapsed  = 0          # ms elapsed in current step
    total_steps   = len(TEST_SEQUENCE) - 1  # exclude DONE sentinel
    current_label, current_vx, current_vy, current_omega, current_dur = TEST_SEQUENCE[0]
    done          = False

    print(f"[TEST] Starting step 1/{total_steps}: {current_label}")

    # ── Main loop ─────────────────────────────────────────────────────────────
    while robot.step(TIMESTEP) != -1:

        if done:
            wheel1.setVelocity(0.0)
            wheel2.setVelocity(0.0)
            wheel3.setVelocity(0.0)
            # still show camera
            image = camera.getImage()
            img = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))[:, :, :3]
            cv2.putText(img, "TEST COMPLETE", (width//2 - 110, height//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 100), 3)
            cv2.imshow("Omni Robot Test", img)
            cv2.waitKey(1)
            continue

        # -- Advance step timer -----------------------------------------------
        step_elapsed += TIMESTEP
        if step_elapsed >= current_dur:
            step_idx    += 1
            step_elapsed = 0

            if step_idx >= len(TEST_SEQUENCE) or TEST_SEQUENCE[step_idx][0] == "DONE":
                print("[TEST] All steps complete.")
                done = True
                continue

            current_label, current_vx, current_vy, current_omega, current_dur = TEST_SEQUENCE[step_idx]
            print(f"[TEST] Step {step_idx + 1}/{total_steps}: {current_label}")

        # -- IK ---------------------------------------------------------------
        speeds = kiwi_ik(current_vx, current_vy, current_omega, WHEEL_ANGLES_DEG)
        speeds = clamp_speeds(speeds, MAX_SPEED)
        w1, w2, w3 = speeds

        wheel1.setVelocity(w1)
        wheel2.setVelocity(w2)
        wheel3.setVelocity(w3)

        # -- Sensors ----------------------------------------------------------
        _, _, yaw = imu.getRollPitchYaw()
        yaw_deg   = np.degrees(yaw)
        encoders  = [enc1.getValue(), enc2.getValue(), enc3.getValue()]

        # -- Camera + overlay -------------------------------------------------
        image = camera.getImage()
        img = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))[:, :, :3]
        img = draw_overlay(img, current_label, step_idx + 1, total_steps,
                           current_vx, current_vy, current_omega,
                           w1, w2, w3, yaw_deg, encoders)
        cv2.imshow("Omni Robot Test", img)
        cv2.waitKey(1)

        # -- Console heartbeat (every ~500ms) ---------------------------------
        if step_elapsed % 512 < TIMESTEP:
            print(f"  yaw={yaw_deg:+.1f}°  enc=[{encoders[0]:.2f}, {encoders[1]:.2f}, {encoders[2]:.2f}]"
                  f"  wheels=[{w1:+.2f}, {w2:+.2f}, {w3:+.2f}]")

    cv2.destroyAllWindows()