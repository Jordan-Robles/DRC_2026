"""
driver.py — Autonomous lane-following driver for a heading-locked kiwi-drive
            robot, fed by a real stitched 360-degree IPM camera image.

ARCHITECTURE
  This module is vision-only. It knows nothing about yaw, world position, or
  serial protocols. Given one IPM frame, it returns a body-frame velocity:

      v_forward, v_right, debug_info = get_motion_command(robot_state, frame)

  v_forward / v_right are in the robot's own fixed body frame (forward = the
  direction the chassis physically points, right = 90 deg clockwise from
  that). Because the chassis never yaws, this body frame is constant for the
  life of the run -- there is no heading to track frame-to-frame.

  Keeping rotation OUT of this file is deliberate: heading-hold (cancelling
  unwanted spin using the Arduino's MPU yaw) is a separate, faster control
  loop and belongs in the bridge script (see pi_bridge.py), not mixed in
  with the vision pipeline. driver.py never returns a rotation command.

WHAT CHANGED FROM THE SIM VERSION
  - No more world x,y / dynamic-heading tracking. The simulator's body also
    never yawed, but driver.py used to *infer* a "forward" direction each
    frame from world-position deltas. A real robot has no ground-truth
    position, so that mechanism is gone. "Forward" is now a one-time
    calibration constant (FORWARD_BEARING_DEG_IN_IMAGE, below) -- you set it
    once after your stitcher is running, and it never changes again.
  - Removed the pygame-specific (W,H,3) transpose. Real frames from an
    OpenCV-based stitching pipeline are already (H,W,3). If you still want
    to test this file against the old sim, set PYGAME_WH_SWAPPED = True.
  - Wall-detection geometry (dead zone, standoff target) is now expressed as
    a *fraction* of the image's half-diagonal instead of a fixed pixel
    count, so it survives a change in stitched-image resolution.
  - The single-wall search timeout is wall-clock (seconds) instead of a
    frame count, since a real camera/stitch/process loop won't run at a
    fixed, known frame rate the way the sim's pygame loop did.
  - HSV thresholds are left as placeholders -- you said you'll retune these
    against real footage, so they're untouched structurally but will need
    new numbers.

CALIBRATION YOU STILL NEED TO DO ON THE BENCH
  1. FORWARD_BEARING_DEG_IN_IMAGE -- which way "forward" points in your
     stitched image's pixel coordinates.
  2. LATERAL_SIGN -- flip to -1.0 if the car steers toward the wrong wall.
  3. INPUT_IS_BGR / PYGAME_WH_SWAPPED -- match your actual frame format.
  4. YELLOW_HSV_*, BLUE_HSV_* -- retune against real footage (you're
     handling this separately).
  5. CENTRE_DEAD_ZONE_FRAC, MIN_PIXELS, LATERAL_GAIN, SINGLE_WALL_TARGET_
     FRACTION -- re-tune once you can see real wall masks; the numbers
     carried over from the sim are just a reasonable starting point.
"""

import time
import math

import numpy as np
import cv2

# ─────────────────────────────────────────────────────────────────────────────
# FRAME FORMAT  (set these to match your real stitching pipeline)
# ─────────────────────────────────────────────────────────────────────────────

# Most OpenCV-based capture/stitch pipelines hand you BGR uint8 frames
# natively. Set False if your stitcher gives you RGB instead.
INPUT_IS_BGR = True

# Only relevant if you're still testing this file against the old pygame
# sim, which returns arrays as (W,H,3) instead of the normal (H,W,3).
# Leave False for real camera input.
PYGAME_WH_SWAPPED = False

# ─────────────────────────────────────────────────────────────────────────────
# CALIBRATION: image pixel-space -> robot body-frame
# ─────────────────────────────────────────────────────────────────────────────
#
# The chassis is heading-locked, so the camera rig never rotates relative to
# the body. That means there is exactly ONE fixed angle, in image pixel
# coordinates (x = column/right, y = row/down), that corresponds to the
# robot's physical "forward" direction -- and once you find it, it never
# changes. Find it empirically once your stitcher is producing real frames:
# e.g. note which camera is mounted facing the direction you want to call
# "front", and read off its bearing in the stitched panorama, or just point
# the chassis at a known wall and see which way wall pixels sit.
#
# 0 = forward points along the image's +x axis (right)
# 90 = forward points along the image's +y axis (down)
# (increasing clockwise, matching standard image/array axis convention)
FORWARD_BEARING_DEG_IN_IMAGE = 0.0

# Flip to -1.0 on the bench if the car corrects toward the wrong wall
# (steers right when it should steer left, or vice versa). Leave at 1.0
# until you've actually watched it do the wrong thing once.
LATERAL_SIGN = 1.0

_fwd_rad = math.radians(FORWARD_BEARING_DEG_IN_IMAGE)
# Unit vector pointing "forward" in image pixel coordinates.
_FWD_IMG = (math.cos(_fwd_rad), math.sin(_fwd_rad))
# Unit vector pointing "right" (90 deg clockwise from forward, in image
# coordinates where y increases downward).
_RGT_IMG = (-math.sin(_fwd_rad) * LATERAL_SIGN, math.cos(_fwd_rad) * LATERAL_SIGN)


def _to_body_frame(dx, dy):
    """
    Project an image-pixel-space offset (dx, dy) onto the robot's fixed
    body frame, returning (forward_component, right_component).
    This is the only place image orientation and body orientation meet --
    everything below this line works purely in body-frame units.
    """
    fwd = dx * _FWD_IMG[0] + dy * _FWD_IMG[1]
    right = dx * _RGT_IMG[0] + dy * _RGT_IMG[1]
    return fwd, right


# ─────────────────────────────────────────────────────────────────────────────
# PARAMETERS  (carried over from the sim version -- re-tune against real
# footage and real geometry; treat these as starting points, not truth)
# ─────────────────────────────────────────────────────────────────────────────

ROBOT_SPEED = 0.8  # output magnitude, in whatever units the bridge expects

YELLOW_HSV_LOW  = np.array([ 20, 100, 100])
YELLOW_HSV_HIGH = np.array([ 40, 255, 255])
BLUE_HSV_LOW    = np.array([100, 100, 100])
BLUE_HSV_HIGH   = np.array([130, 255, 255])

# Minimum pixels to consider a wall visible
MIN_PIXELS = 50

# Dead zone: ignore pixels within this fraction of the image's half-diagonal
# (this is the resolution-independent replacement for the old fixed-pixel
# CENTRE_DEAD_ZONE_PX = 55, which only made sense at the sim's 600x600 image)
CENTRE_DEAD_ZONE_FRAC = 55.0 / 600.0

# How strongly lateral error drives correction
LATERAL_GAIN = 4

# Minimum forward component (fraction of ROBOT_SPEED)
MIN_FORWARD = 0.2

# Target distance from wall as fraction of max visible radius (single wall mode)
SINGLE_WALL_TARGET_FRACTION = 0.45

# Seconds to search for a lost wall before committing to single-wall corner
# mode. Wall-clock based (not frame-count) since a real vision loop's frame
# rate isn't fixed or known in advance the way the sim's was.
SEARCH_TIMEOUT_SEC = 2.0

# Nudge strength when searching for lost wall
SEARCH_NUDGE = 1.0

# ─────────────────────────────────────────────────────────────────────────────
# STATE
# ─────────────────────────────────────────────────────────────────────────────

_last_v_forward = 0.0
_last_v_right   = 0.0

# State machine
_drive_state    = 'both'
_search_deadline = None  # time.monotonic() timestamp, or None when not searching

# Last known body-frame direction TO yellow wall and AWAY from blue wall
# (unit vectors). Used to maintain a consistent push direction when only one
# wall is visible, and persisted across frames since a wall can disappear.
_toward_yellow_fwd = None
_toward_yellow_right = None
_away_from_blue_fwd = None
_away_from_blue_right = None

# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def _wall_vector(mask, cx, cy, dead_zone):
    """
    Find the mean position of wall pixels as a (dx, dy) vector from image
    centre, in raw image pixel coordinates. Returns (dx, dy, count).
    Pixels within dead_zone radius are ignored.
    """
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        return 0.0, 0.0, 0

    dx = xs - cx
    dy = ys - cy
    radii = np.sqrt(dx**2 + dy**2)

    valid = radii > dead_zone
    dx, dy, radii = dx[valid], dy[valid], radii[valid]
    count = len(dx)
    if count < MIN_PIXELS:
        return 0.0, 0.0, 0

    mean_dx = float(np.mean(dx))
    mean_dy = float(np.mean(dy))
    return mean_dx, mean_dy, count


def _unit(dx, dy):
    mag = math.hypot(dx, dy)
    if mag < 1e-6:
        return 0.0, 0.0
    return dx / mag, dy / mag


def _normalise(vx, vy, speed):
    mag = math.hypot(vx, vy)
    if mag < 1e-6:
        return 0.0, 0.0
    return vx / mag * speed, vy / mag * speed


# ─────────────────────────────────────────────────────────────────────────────
# MAIN ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def get_motion_command(robot_state, camera_view_data):
    """
    robot_state: dict, currently unused by the core algorithm (kept for API
                 compatibility / future use, e.g. arrow markers). Pass {} if
                 you have nothing to put in it.
    camera_view_data: (H, W, 3) uint8 array, the stitched 360-degree IPM
                 frame, robot-centred. BGR by default (see INPUT_IS_BGR).

    Returns (v_forward, v_right, debug_info):
      v_forward, v_right -- body-frame velocity command. forward/right are
        defined by FORWARD_BEARING_DEG_IN_IMAGE above; they do NOT
        necessarily match whatever axis convention your Arduino firmware
        expects (e.g. this codebase's kiwi_autonomous.ino calls its two
        translation axes "strafe X" / "fwd-back Y") -- map forward/right
        onto your hardware's x/y in the bridge script, not here.
      debug_info -- dict of internal state for logging/tuning. No image is
        attached (unlike the old sim version) to avoid copying full-res
        frames every call; pass camera_view_data through yourself if you
        want to draw an overlay.
    """
    global _last_v_forward, _last_v_right
    global _drive_state, _search_deadline
    global _toward_yellow_fwd, _toward_yellow_right
    global _away_from_blue_fwd, _away_from_blue_right

    # ── Prepare image ────────────────────────────────────────────────────────
    frame = camera_view_data
    if PYGAME_WH_SWAPPED:
        frame = frame.transpose(1, 0, 2)

    if INPUT_IS_BGR:
        img_bgr = frame
    else:
        img_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    img_h, img_w = img_hsv.shape[:2]
    cx, cy  = img_w / 2.0, img_h / 2.0
    max_r   = math.hypot(cx, cy)
    dead_zone_px = CENTRE_DEAD_ZONE_FRAC * max_r

    yellow_mask = cv2.inRange(img_hsv, YELLOW_HSV_LOW, YELLOW_HSV_HIGH)
    blue_mask   = cv2.inRange(img_hsv, BLUE_HSV_LOW,   BLUE_HSV_HIGH)

    # ── Get wall vectors (raw image pixel coords) ───────────────────────────
    ydx, ydy, y_count = _wall_vector(yellow_mask, cx, cy, dead_zone_px)
    bdx, bdy, b_count = _wall_vector(blue_mask,   cx, cy, dead_zone_px)

    have_yellow = y_count >= MIN_PIXELS
    have_blue   = b_count >= MIN_PIXELS
    have_both   = have_yellow and have_blue

    # Project into body frame once, here, so every state below works purely
    # in (forward, right) units and never touches image coordinates again.
    y_fwd, y_right = _to_body_frame(ydx, ydy)
    b_fwd, b_right = _to_body_frame(bdx, bdy)

    # Update stored wall directions whenever walls are clearly visible
    if have_yellow:
        _toward_yellow_fwd, _toward_yellow_right = _unit(y_fwd, y_right)
    if have_blue:
        _away_from_blue_fwd, _away_from_blue_right = _unit(-b_fwd, -b_right)

    now = time.monotonic()

    # ── State transitions ────────────────────────────────────────────────────
    if have_both:
        _drive_state = 'both'
        _search_deadline = None
    elif have_yellow and not have_blue:
        if _drive_state in ('both', 'searching_yellow', 'corner_blue'):
            _drive_state = 'searching_blue'
            _search_deadline = now + SEARCH_TIMEOUT_SEC
        elif _drive_state == 'searching_blue':
            if _search_deadline is not None and now >= _search_deadline:
                _drive_state = 'corner_yellow'
    elif have_blue and not have_yellow:
        if _drive_state in ('both', 'searching_blue', 'corner_yellow'):
            _drive_state = 'searching_yellow'
            _search_deadline = now + SEARCH_TIMEOUT_SEC
        elif _drive_state == 'searching_yellow':
            if _search_deadline is not None and now >= _search_deadline:
                _drive_state = 'corner_blue'
    else:
        debug_info = {
            'have_yellow': False, 'have_blue': False, 'have_both': False,
            'y_count': 0, 'b_count': 0,
            'cmd_v_forward': _last_v_forward, 'cmd_v_right': _last_v_right,
            'state': 'no_walls',
            'dead_zone_px': dead_zone_px, 'img_w': img_w, 'img_h': img_h,
        }
        return _last_v_forward, _last_v_right, debug_info

    # ── Compute correction (all in body-frame units; forward = (1, 0)) ──────

    if _drive_state == 'both':
        mid_fwd   = (y_fwd + b_fwd) / 2.0
        mid_right = (y_right + b_right) / 2.0
        raw_x = 1.0 + (mid_fwd / max_r) * LATERAL_GAIN
        raw_y = (mid_right / max_r) * LATERAL_GAIN

    elif _drive_state == 'searching_blue':
        y_dist = math.hypot(y_fwd, y_right)
        target = SINGLE_WALL_TARGET_FRACTION * max_r
        standoff_error = (y_dist - target) / max_r
        away_fwd   = -_toward_yellow_fwd if _toward_yellow_fwd is not None else 0.0
        away_right = -_toward_yellow_right if _toward_yellow_right is not None else 0.0
        push = standoff_error * LATERAL_GAIN + SEARCH_NUDGE
        raw_x = 1.0 + away_fwd * push
        raw_y = away_right * push

    elif _drive_state == 'searching_yellow':
        b_dist = math.hypot(b_fwd, b_right)
        target = SINGLE_WALL_TARGET_FRACTION * max_r
        standoff_error = (target - b_dist) / max_r
        away_fwd   = _away_from_blue_fwd if _away_from_blue_fwd is not None else 0.0
        away_right = _away_from_blue_right if _away_from_blue_right is not None else 0.0
        push = standoff_error * LATERAL_GAIN + SEARCH_NUDGE
        raw_x = 1.0 + away_fwd * push
        raw_y = away_right * push

    elif _drive_state == 'corner_yellow':
        y_dist = math.hypot(y_fwd, y_right)
        target = SINGLE_WALL_TARGET_FRACTION * max_r
        standoff_error = (y_dist - target) / max_r
        away_fwd   = -_toward_yellow_fwd if _toward_yellow_fwd is not None else 0.0
        away_right = -_toward_yellow_right if _toward_yellow_right is not None else 0.0
        raw_x = 1.0 + away_fwd * standoff_error * LATERAL_GAIN
        raw_y = away_right * standoff_error * LATERAL_GAIN

    elif _drive_state == 'corner_blue':
        b_dist = math.hypot(b_fwd, b_right)
        target = SINGLE_WALL_TARGET_FRACTION * max_r
        standoff_error = (target - b_dist) / max_r
        away_fwd   = _away_from_blue_fwd if _away_from_blue_fwd is not None else 0.0
        away_right = _away_from_blue_right if _away_from_blue_right is not None else 0.0
        raw_x = 1.0 + away_fwd * standoff_error * LATERAL_GAIN
        raw_y = away_right * standoff_error * LATERAL_GAIN

    else:
        debug_info = {
            'have_yellow': have_yellow, 'have_blue': have_blue, 'have_both': False,
            'y_count': y_count, 'b_count': b_count,
            'cmd_v_forward': _last_v_forward, 'cmd_v_right': _last_v_right,
            'state': 'fallthrough',
            'dead_zone_px': dead_zone_px, 'img_w': img_w, 'img_h': img_h,
        }
        return _last_v_forward, _last_v_right, debug_info

    # ── Minimum forward component ────────────────────────────────────────────
    # In body frame, forward is trivially (1, 0), so the forward component
    # of (raw_x, raw_y) is just raw_x.
    if raw_x < MIN_FORWARD:
        raw_x = MIN_FORWARD

    v_forward, v_right = _normalise(raw_x, raw_y, ROBOT_SPEED)
    _last_v_forward, _last_v_right = v_forward, v_right

    debug_info = {
        'have_yellow': have_yellow, 'have_blue': have_blue, 'have_both': have_both,
        'y_count': y_count, 'b_count': b_count,
        'cmd_v_forward': v_forward, 'cmd_v_right': v_right,
        'state': _drive_state,
        'dead_zone_px': dead_zone_px, 'img_w': img_w, 'img_h': img_h,
    }

    return v_forward, v_right, debug_info