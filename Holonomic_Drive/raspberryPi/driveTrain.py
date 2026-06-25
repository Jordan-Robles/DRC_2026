"""
drivetrain.py — Converts driver.py's body-frame velocity command into
actuation for a front-wheel-drive, rear-wheel-steer ("tricycle") chassis.

This is the ONLY place that needs to change when swapping drivetrains.
driver.py stays exactly as-is: it always outputs a body-frame
(v_forward, v_right) vector describing "the direction the vision pipeline
wants to go" -- it has no opinion about wheels, and never did. For the
holonomic kiwi chassis that vector was fed straight into a 3-wheel mix.
For this chassis there's no sideways motion available, so the lateral
(v_right) component has to become a STEERING ANGLE instead of a velocity.

NOTE ON HEADING: with a holonomic, heading-locked chassis, any unwanted
rotation was drift to be cancelled. With this chassis, rotation IS how you
change direction -- there's no such thing as "drift" to suppress anymore,
the rear wheel steering the chassis around is the whole mechanism. That's
why pi_bridge.py no longer locks onto a fixed heading setpoint.
"""

import math

# Maximum mechanical steering deflection of the rear wheel, in degrees.
# CALIBRATE: set to whatever your steering linkage can physically do before
# binding, with a few degrees of margin.
MAX_STEER_DEG = 30.0

# How aggressively lateral error becomes steering angle. Start small (e.g.
# 0.5) and increase until cornering feels responsive without overshoot/
# oscillation.
STEER_GAIN = 0.5

# Optional extra steering term from the rx channel (e.g. a yaw-rate damping
# term computed from the MPU in pi_bridge.py). Leave at 0.0 until/unless you
# wire something into rx -- driver.py never produces this on its own.
RX_STEER_GAIN = 0.0

# Derate drive speed the harder you're steering, the way a real car
# naturally slows into a sharp turn. 0.0 disables this (always drive at the
# full requested speed regardless of steering angle); 1.0 means speed goes
# to zero at max steering lock.
TURN_SPEED_DERATE = 0.4


def mix_to_tricycle(v_forward, v_right, rx=0.0):
    """
    v_forward, v_right: driver.py's body-frame output, unchanged.
    rx: optional extra steering bias (e.g. yaw-rate damping). Pass 0.0 if
        you're not feeding anything into this channel.

    Returns (drive_speed, steer_deg):
      drive_speed -- commanded speed for BOTH front wheels together (no
        differential -- in this layout they only ever drive, they don't
        steer).
      steer_deg -- commanded rear-wheel steering angle in degrees, where 0
        is straight ahead and positive turns the chassis to its right.
    """
    drive_speed = math.hypot(v_forward, v_right)

    # driver.py's MIN_FORWARD clamp guarantees v_forward >= 0 in normal
    # operation, but atan2(y, x) flips to +/-180 deg for any negative x even
    # with y = 0 -- e.g. atan2(0, -0.5) = 180 deg, not the "no turn needed"
    # you'd want for driving straight backward. Clamp defensively so this
    # function stays sane even if that invariant ever changes upstream.
    safe_forward = max(v_forward, 0.0)
    heading_offset_rad = math.atan2(v_right, safe_forward)
    steer_deg = math.degrees(heading_offset_rad) * STEER_GAIN
    steer_deg += rx * RX_STEER_GAIN
    steer_deg = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer_deg))

    derate = 1.0 - TURN_SPEED_DERATE * (abs(steer_deg) / MAX_STEER_DEG)
    drive_speed *= max(0.0, derate)

    return drive_speed, steer_deg