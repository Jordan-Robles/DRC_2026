"""
pi_bridge.py — Runs on the Pi. Ties together camera capture, driver.py, and
               the Arduino serial link. This is the only place that talks to
               kiwi_autonomous.ino.

WHAT THIS DOES, IN ORDER, EVERY LOOP ITERATION
  1. Read the latest yaw line the Arduino has streamed over serial.
  2. Feed that yaw into a heading-hold controller, which outputs a small
     rotation command (rx) whose only job is to cancel drift away from a
     locked heading setpoint. This is what keeps "the cameras/body never
     rotate" true in practice, since nothing about the mechanical chassis
     physically prevents it from spinning -- the MPU + this loop is what
     holds it still.
  3. Grab the latest stitched 12-camera IPM frame (you need to fill this
     in -- see get_stitched_ipm_frame() below).
  4. Call driver.get_motion_command() to get a vision-based (v_forward,
     v_right) translation command.
  5. Map (v_forward, v_right, rx) onto the Arduino's wire format and send
     "x,y,rx\\n" over serial.

This matches the protocol already described in kiwi_autonomous.ino's header
comment: "Pi -> Arduino: vx,vy,rx ... the Pi computes rx itself from the
yaw below." No Arduino firmware changes are required for this script to
work as written.

A NOTE ON WHERE HEADING-HOLD LIVES
  Doing the correction here means the control loop's speed is capped by
  however fast your camera-grab + stitch + vision pipeline runs, since yaw
  read, rx compute, and serial write all happen in the same loop iteration
  as the vision step. If 12-camera stitching turns out to be slow (tens of
  ms per frame) and you see the chassis hunting or drifting more than you'd
  like before the correction catches it, the fix is to move heading-hold
  into the Arduino itself -- it already has the yaw at full MPU update
  rate, independent of how slow vision is. That's a firmware change, not
  something this script can do alone. Worth keeping in mind if bench
  testing shows the locked heading isn't holding tightly enough.

CALIBRATION YOU STILL NEED TO DO ON THE BENCH
  - SERIAL_PORT: your actual port (e.g. "/dev/ttyUSB0", "/dev/ttyACM0", or
    a COM port on Windows).
  - STRAFE_SIGN / FORWARD_SIGN: kiwi_autonomous.ino's own comments label its
    two translation axes "Ch1 (strafe X)" and "Ch2 (fwd/back Y)" -- that is
    very likely NOT the same axis order/sign as driver.py's (v_forward,
    v_right). Command pure forward on the bench, watch which way the chassis
    actually moves, and flip these two constants until it matches.
  - HEADING_HOLD_KP (and KI if you add it): start small and increase until
    drift is cancelled without oscillating.
  - MAX_RX: keep this small -- it should only ever be correcting drift, not
    performing deliberate turns.
"""

import sys
import time
import threading

import serial

import driver

# ─────────────────────────────────────────────────────────────────────────────
# SERIAL LINK
# ─────────────────────────────────────────────────────────────────────────────

SERIAL_PORT = "/dev/ttyACM0"   # CALIBRATE: your actual port
SERIAL_BAUD = 115200            # must match Serial.begin(115200) in the .ino

# kiwi_autonomous.ino's watchdog stops the motors if no command arrives for
# 300ms. Keep comfortably under that even if vision is slow some frames.
ARDUINO_COMMAND_TIMEOUT_S = 0.3

# ─────────────────────────────────────────────────────────────────────────────
# AXIS MAPPING  (driver.py's body frame -> the Arduino's wire format)
# ─────────────────────────────────────────────────────────────────────────────
#
# driver.py returns (v_forward, v_right) in its own calibrated body frame.
# kiwi_autonomous.ino's comments call its two translation inputs "strafe X"
# and "fwd/back Y" -- so the mapping is (roughly):
#   arduino_x  <-  v_right   (strafe)
#   arduino_y  <-  v_forward (fwd/back)
# but the SIGN of each is whatever your wiring/wheel layout happens to
# produce, and that can only be confirmed on the bench. Flip these to -1.0
# as needed once you've watched the chassis respond to a known command.
STRAFE_SIGN  = 1.0
FORWARD_SIGN = 1.0

def to_arduino_xy(v_forward, v_right):
    x = STRAFE_SIGN * v_right
    y = FORWARD_SIGN * v_forward
    return x, y

# ─────────────────────────────────────────────────────────────────────────────
# HEADING HOLD
# ─────────────────────────────────────────────────────────────────────────────

# How hard to correct per degree of heading error. Start small.
HEADING_HOLD_KP = 0.01

# Hard cap on the rotation command. This loop should only ever be cancelling
# small drift, never performing an intentional turn -- keep it tight.
MAX_RX = 0.3

# How long to let the MPU settle after boot before locking the setpoint.
# (kiwi_autonomous.ino already does its own gyro-offset calibration in
# testMPU() during setup() before it ever starts streaming yaw, so this is
# just extra settle time on the Pi side, not a substitute for that.)
SETTLE_S = 1.0


def shortest_angle_diff(target_deg, current_deg):
    """Signed shortest-path difference, handling the 0/360 wraparound."""
    return ((target_deg - current_deg + 180.0) % 360.0) - 180.0


class HeadingHold:
    """
    Locks onto whatever heading the chassis is at when lock_in() is called,
    then on every update() returns a small rx correction pulling back
    toward that locked heading.

    NOTE: the MPU6050 has no magnetometer, so its yaw is gyro-integrated
    and will drift slowly over long runs (minutes+). That's fine for THIS
    purpose -- cancelling short-term unwanted rotation while driving a lap
    -- but don't treat it as an absolute compass heading over a long
    session. If you need that, you'd want sensor fusion with a
    magnetometer (e.g. an MPU9250) instead.
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

def send_command(ser, x, y, rx):
    x = max(-1.0, min(1.0, x))
    y = max(-1.0, min(1.0, y))
    rx = max(-1.0, min(1.0, rx))
    ser.write(f"{x:.3f},{y:.3f},{rx:.3f}\n".encode("ascii"))


def main():
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.05)

    yaw_reader = YawReader(ser)
    yaw_reader.start()

    heading_hold = HeadingHold()

    print(f"[pi_bridge] settling {SETTLE_S}s before locking heading...")
    time.sleep(SETTLE_S)

    yaw, age = yaw_reader.latest()
    if yaw is None:
        print("[pi_bridge] no yaw received from Arduino -- check wiring/port.")
        send_command(ser, 0.0, 0.0, 0.0)
        ser.close()
        sys.exit(1)

    heading_hold.lock_in(yaw)
    print(f"[pi_bridge] heading locked at {yaw:.1f} deg")

    try:
        while True:
            frame = get_stitched_ipm_frame()

            yaw, age = yaw_reader.latest()
            rx = heading_hold.update(yaw) if age < ARDUINO_COMMAND_TIMEOUT_S else 0.0

            if frame is not None:
                v_forward, v_right, debug_info = driver.get_motion_command({}, frame)
                arduino_x, arduino_y = to_arduino_xy(v_forward, v_right)
                send_command(ser, arduino_x, arduino_y, rx)
            else:
                # No frame this iteration -- still send the heading-hold
                # correction with zero translation rather than going
                # silent (silence past ARDUINO_COMMAND_TIMEOUT_S anyway
                # trips the Arduino's own watchdog and stops the motors).
                send_command(ser, 0.0, 0.0, rx)

    except KeyboardInterrupt:
        pass
    finally:
        print("[pi_bridge] stopping.")
        send_command(ser, 0.0, 0.0, 0.0)
        yaw_reader.stop()
        ser.close()


if __name__ == "__main__":
    main()