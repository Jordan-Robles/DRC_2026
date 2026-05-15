"""
kiwi_controller.py
==================
Kiwi drive controller — simple Robot (no Supervisor).
Bouncing and rotation are handled by physics damping in the .wbt.
"""

import math
from dataclasses import dataclass
from controller import Robot

MODE        = "teleop"
WHEEL_R     = 0.035
BASE_R      = 0.138
MAX_SPEED   = 30.0
DRIVE_SPEED = 1.0

DRIVE_ANGLES = [math.radians(270), math.radians(30), math.radians(150)]


@dataclass
class MotionCommand:
    vx:    float = 0.0   # m/s forward
    vy:    float = 0.0   # m/s left
    omega: float = 0.0   # ignored


def kiwi_ik(cmd):
    vx, vy = cmd.vx, cmd.vy
    speeds = [
        (math.sin(t) * vx - math.cos(t) * vy) / WHEEL_R
        for t in DRIVE_ANGLES
    ]
    peak = max(abs(s) for s in speeds)
    if peak > MAX_SPEED:
        speeds = [s * MAX_SPEED / peak for s in speeds]
    return speeds


def teleop_source(keyboard):
    vx_raw, vy_raw = 0.0, 0.0
    key = keyboard.getKey()
    pressed = set()
    while key != -1:
        pressed.add(key)
        key = keyboard.getKey()
    if ord('W') in pressed: vx_raw += 1.0
    if ord('S') in pressed: vx_raw -= 1.0
    if ord('A') in pressed: vy_raw += 1.0
    if ord('D') in pressed: vy_raw -= 1.0
    mag = math.hypot(vx_raw, vy_raw)
    if mag > 1.0:
        vx_raw /= mag
        vy_raw /= mag
    return MotionCommand(vx=vx_raw * DRIVE_SPEED, vy=vy_raw * DRIVE_SPEED)


def autonomous_source(sensors):
    """
    ─────────────────────────────────────────────────────
    PLUG YOUR AUTONOMOUS CODE IN HERE
    sensors: {"gps": (x,y,z), "gyro": (wx,wy,wz), "time": float}
    Return MotionCommand(vx, vy) — vx=forward, vy=left, m/s
    ─────────────────────────────────────────────────────
    """
    # --- REPLACE BELOW ---
    return MotionCommand(vx=0.2, vy=0.0)
    # --- END REPLACE ---


def main():
    robot    = Robot()
    timestep = int(robot.getBasicTimeStep())

    motors = [robot.getDevice(f"wheel{i}") for i in range(3)]
    for m in motors:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)

    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)

    gps = robot.getDevice("gps")
    gps.enable(timestep)

    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    print("=" * 55)
    print("  KIWI DRIVE  —  physics damped")
    print(f"  Mode: {MODE.upper()}")
    print("  W=forward  S=back  A=left  D=right")
    print("=" * 55)

    step = 0
    while robot.step(timestep) != -1:
        sensors = {
            "gps":  gps.getValues(),
            "gyro": gyro.getValues(),
            "time": robot.getTime(),
        }
        if MODE == "teleop":
            cmd = teleop_source(keyboard)
        else:
            cmd = autonomous_source(sensors)

        speeds = kiwi_ik(cmd)
        for m, s in zip(motors, speeds):
            m.setVelocity(s)

        step += 1
        if step % 60 == 0:
            p = sensors["gps"]
            print(f"t={sensors['time']:6.2f}s  "
                  f"pos=({p[0]:+.3f},{p[1]:+.3f})  "
                  f"cmd=(vx={cmd.vx:+.3f} vy={cmd.vy:+.3f})  "
                  f"w=[{','.join(f'{s:+.1f}' for s in speeds)}]")


if __name__ == "__main__":
    main()
