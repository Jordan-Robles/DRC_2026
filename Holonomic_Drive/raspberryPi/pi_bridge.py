"""
pi_bridge.py — Runs on the Pi. Ties together camera capture, driver.py, and
               the Arduino serial link. Talks to a front-wheel-drive,
               rear-wheel-steer ("tricycle") chassis -- see drivetrain.py
               and tricycle_autonomous.ino.

WHAT THIS DOES, IN ORDER, EVERY LOOP ITERATION
  1. Grab the latest stitched 12-camera IPM frame (you need to fill this
     in -- see get_stitched_ipm_frame() below).
  2. Call driver.get_motion_command() to get a vision-based (v_forward,
     v_right) body-frame command. driver.py is completely unaware of, and
     unaffected by, what kind of chassis is on the other end of this.
  3. Pass (v_forward, v_right) into drivetrain.mix_to_tricycle(), which
     converts the body-frame vector into (drive_speed, steer_deg) for two
     front drive wheels and one rear steering wheel.
  4. Send "drive,steer\\n" over serial.

WHY THERE'S NO HEADING-HOLD HERE ANYMORE
  The previous (holonomic, heading-locked) version of this script used the
  MPU's yaw to cancel unwanted rotation, because that chassis was only
  supposed to translate, never turn. This chassis is the opposite: turning
  is the ONLY way it changes direction, so there's nothing to "lock" --
  forcing a fixed heading would actively fight the steering. The yaw stream
  is still read (YawReader, below) since it's useful telemetry and a
  building block if you later want closed-loop steering (e.g. damping
  oscillation with a yaw-rate term fed into drivetrain.mix_to_tricycle's
  optional `rx` argument), but nothing currently locks onto it.

CALIBRATION YOU STILL NEED TO DO ON THE BENCH
  - SERIAL_PORT: your actual port (e.g. "/dev/ttyUSB0", "/dev/ttyACM0", or
    a COM port on Windows).
  - Everything in drivetrain.py: MAX_STEER_DEG, STEER_GAIN, TURN_SPEED_
    DERATE. Command a known lateral error on the bench and watch the
    chassis steer the correct way before trusting it on the track.
"""

import time
import threading

import serial

import driver
import driveTrain

# ─────────────────────────────────────────────────────────────────────────────
# SERIAL LINK
# ─────────────────────────────────────────────────────────────────────────────

SERIAL_PORT = "/dev/ttyACM0"   # CALIBRATE: your actual port
SERIAL_BAUD = 115200            # must match Serial.begin(115200) in the .ino

# kiwi_autonomous.ino's watchdog stops the motors if no command arrives for
# 300ms. Keep comfortably under that even if vision is slow some frames.
ARDUINO_COMMAND_TIMEOUT_S = 0.3

# Translation/steering mixing now lives entirely in drivetrain.py -- see
# drivetrain.mix_to_tricycle(), called from the main loop below. Nothing
# chassis-specific belongs in this file beyond "call the mixer and send the
# result."

# ─────────────────────────────────────────────────────────────────────────────
# HEADING HOLD -- NOT USED BY DEFAULT ON THIS CHASSIS
# ─────────────────────────────────────────────────────────────────────────────
# Kept as a building block, not wired into main() below. A heading-locked
# holonomic chassis used this to cancel drift; this chassis steers on
# purpose, so locking a fixed setpoint would fight the steering. If you
# later want closed-loop steering refinement (e.g. damping oscillation with
# a yaw-RATE term, not a fixed yaw setpoint), this class is the wrong shape
# for that -- it locks an absolute heading -- but shortest_angle_diff()
# below is still the right building block for any yaw-error math you do.

HEADING_HOLD_KP = 0.01
MAX_RX = 0.3


def shortest_angle_diff(target_deg, current_deg):
    """Signed shortest-path difference, handling the 0/360 wraparound."""
    return ((target_deg - current_deg + 180.0) % 360.0) - 180.0


class HeadingHold:
    """
    Locks onto whatever heading the chassis is at when lock_in() is called,
    then on every update() returns a small correction pulling back toward
    that locked heading. Not used by main() in this configuration -- see
    note above.
    """

    def __init__(self, kp=HEADING_HOLD_KP, max_rx=MAX_RX):
        self.kp = kp
        self.max_rx = max_rx
        self.setpoint_deg = None

    def lock_in(self, current_yaw_deg):
        self.setpoint_deg = current_yaw_deg

    def update(self, current_yaw_deg):
        if self.setpoint_deg is None or current_yaw_deg is None:
            return 0.0
        error = shortest_angle_diff(self.setpoint_deg, current_yaw_deg)
        rx = self.kp * error
        return max(-self.max_rx, min(self.max_rx, rx))


# ─────────────────────────────────────────────────────────────────────────────
# YAW READER  (background thread, non-blocking)
# ─────────────────────────────────────────────────────────────────────────────

class YawReader:
    """
    Continuously reads "yaw\\n" lines streamed by the Arduino on its own
    schedule (~50Hz per the .ino's YAW_SEND_MS) and exposes the latest
    value. Runs in a background thread so a slow vision loop never causes
    the serial read buffer to back up.
    """

    def __init__(self, ser):
        self._ser = ser
        self._lock = threading.Lock()
        self._yaw = None
        self._last_update = 0.0
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True

    def _run(self):
        while not self._stop:
            try:
                line = self._ser.readline()
                if not line:
                    continue
                text = line.decode("ascii", errors="ignore").strip()
                if not text:
                    continue
                yaw = float(text)
                with self._lock:
                    self._yaw = yaw
                    self._last_update = time.monotonic()
            except (ValueError, serial.SerialException):
                continue

    def latest(self):
        """Returns (yaw_degrees_or_None, age_seconds)."""
        with self._lock:
            if self._yaw is None:
                return None, float("inf")
            return self._yaw, time.monotonic() - self._last_update


# ─────────────────────────────────────────────────────────────────────────────
# CAMERA INPUT  -- fill this in with your real 12-camera capture + stitch
# ─────────────────────────────────────────────────────────────────────────────

def get_stitched_ipm_frame():
    """
    TODO: replace with your real pipeline. Must return a (H, W, 3) uint8
    BGR numpy array -- the stitched 360-degree IPM frame, robot-centred,
    matching whatever resolution/scale you settle on. Return None if a
    frame isn't ready yet (the main loop will skip that iteration and
    reuse the last motion command, same as driver.py's own no-walls
    fallback).

    This is also where INPUT_IS_BGR / PYGAME_WH_SWAPPED in driver.py need
    to actually match reality -- if your capture pipeline gives you RGB
    instead of BGR, either convert here or flip driver.INPUT_IS_BGR.
    """
    raise NotImplementedError(
        "Wire up your 12-camera capture + stitch pipeline here."
    )


# ─────────────────────────────────────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────

def send_command(ser, drive_speed, steer_deg):
    drive_speed = max(-1.0, min(1.0, drive_speed))
    steer_deg = max(-driveTrain.MAX_STEER_DEG, min(driveTrain.MAX_STEER_DEG, steer_deg))
    ser.write(f"{drive_speed:.3f},{steer_deg:.2f}\n".encode("ascii"))


def main():
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.05)

    # Still read yaw for telemetry / future closed-loop steering refinement
    # (see the module docstring), even though nothing locks onto it here.
    yaw_reader = YawReader(ser)
    yaw_reader.start()

    try:
        while True:
            frame = get_stitched_ipm_frame()

            if frame is not None:
                v_forward, v_right, debug_info = driver.get_motion_command({}, frame)
                drive_speed, steer_deg = driveTrain.mix_to_tricycle(v_forward, v_right)
                send_command(ser, drive_speed, steer_deg)
            else:
                # No frame this iteration -- hold position rather than
                # sending nothing (silence past ARDUINO_COMMAND_TIMEOUT_S
                # trips the Arduino's own watchdog and stops the motors).
                send_command(ser, 0.0, 0.0)

    except KeyboardInterrupt:
        pass
    finally:
        print("[pi_bridge] stopping.")
        send_command(ser, 0.0, 0.0)
        yaw_reader.stop()
        ser.close()


if __name__ == "__main__":
    main()