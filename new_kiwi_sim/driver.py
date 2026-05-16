"""
driver.py — Autonomous lane-following driver for the RC car sim.

Drop this file in the same folder as rc_car_sim.py.
In rc_car_sim.py, replace the body of get_motion_command with:

    from driver import get_motion_command as _drive
    return _drive(robot_state, camera_view_data)

How it works:
  - Detects yellow (outer) and blue (inner) wall pixels via HSV masking.
  - Computes the mean position of each wall's pixels as a 2D vector from
    the robot centre in the IPM image.
  - The desired lateral correction is the midpoint between the two wall
    vectors — push toward the midpoint to stay centred.
  - Single-wall fallback: when only one wall is visible, maintain a fixed
    standoff distance from it, in the direction away from that wall.
  - Dynamic heading tracks actual travel direction for forward drive.
  - State machine handles searching for lost walls before committing to
    single-wall corner mode.
"""

import numpy as np
import math
import cv2

# ─────────────────────────────────────────────────────────────────────────────
# PARAMETERS
# ─────────────────────────────────────────────────────────────────────────────

ROBOT_SPEED = 0.5

YELLOW_HSV_LOW  = np.array([ 20, 100, 100])
YELLOW_HSV_HIGH = np.array([ 40, 255, 255])
BLUE_HSV_LOW    = np.array([100, 100, 100])
BLUE_HSV_HIGH   = np.array([130, 255, 255])

# Minimum pixels to consider a wall visible
MIN_PIXELS = 50

# Dead zone: ignore pixels within this radius of centre (robot body)
CENTRE_DEAD_ZONE_PX = 55

# How strongly lateral error drives correction
LATERAL_GAIN = 2.5

# Minimum forward component (fraction of ROBOT_SPEED)
MIN_FORWARD = 0.2

# Target distance from wall as fraction of max visible radius (single wall mode)
SINGLE_WALL_TARGET_FRACTION = 0.45

# Dynamic heading tracking rate
HEADING_TRACK_RATE = 0.15

# Frames to search for lost wall before committing to corner mode
SEARCH_TIMEOUT_FRAMES = 30

# Nudge strength when searching for lost wall
SEARCH_NUDGE = 0.25

# ─────────────────────────────────────────────────────────────────────────────
# STATE
# ─────────────────────────────────────────────────────────────────────────────

_last_vx         = 0.0
_last_vy         = 0.0
_dynamic_heading = None
_last_x          = None
_last_y          = None

# State machine
_drive_state        = 'both'
_search_frames_left = 0

# Last known direction TO yellow wall and AWAY from blue wall (unit vectors)
# These are stored in world frame and updated whenever walls are visible.
# Used to maintain consistent push direction when only one wall is seen.
_toward_yellow_x = None
_toward_yellow_y = None
_away_from_blue_x = None
_away_from_blue_y = None

# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def _wall_vector(mask, cx, cy, dead_zone):
    """
    Find the mean position of wall pixels as a (dx, dy) vector from image centre.
    Returns (dx, dy, count). dx/dy are in pixel units.
    Pixels within dead_zone radius are ignored.
    """
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        return 0.0, 0.0, 0

    dx = xs - cx
    dy = ys - cy
    radii = np.sqrt(dx**2 + dy**2)

    # Remove dead zone
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
    global _last_vx, _last_vy, _dynamic_heading, _last_x, _last_y
    global _drive_state, _search_frames_left
    global _toward_yellow_x, _toward_yellow_y, _away_from_blue_x, _away_from_blue_y

    manual_heading = robot_state.get('heading', 0.0)
    rx = robot_state.get('x', 0.0)
    ry = robot_state.get('y', 0.0)

    # ── Dynamic heading ──────────────────────────────────────────────────────
    if _dynamic_heading is None:
        _dynamic_heading = manual_heading

    if _last_x is not None:
        ddx = rx - _last_x
        ddy = ry - _last_y
        dist = math.hypot(ddx, ddy)
        if dist > 0.5:
            # Reset
            _dynamic_heading    = manual_heading
            _drive_state        = 'both'
            _search_frames_left = 0
            _toward_yellow_x    = None
            _toward_yellow_y    = None
            _away_from_blue_x   = None
            _away_from_blue_y   = None
            _last_x, _last_y    = rx, ry
            return 0.0, 0.0
        if dist > 1e-4:
            travel = math.atan2(ddy, ddx)
            diff = (travel - _dynamic_heading + math.pi) % (2 * math.pi) - math.pi
            _dynamic_heading = (_dynamic_heading + HEADING_TRACK_RATE * diff) % (2 * math.pi)

    _last_x, _last_y = rx, ry
    heading = _dynamic_heading

    # ── Prepare image ────────────────────────────────────────────────────────
    img_rgb = camera_view_data.transpose(1, 0, 2)
    img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    img_h, img_w = img_hsv.shape[:2]
    cx, cy  = img_w / 2.0, img_h / 2.0
    max_r   = math.hypot(cx, cy)

    yellow_mask = cv2.inRange(img_hsv, YELLOW_HSV_LOW, YELLOW_HSV_HIGH)
    blue_mask   = cv2.inRange(img_hsv, BLUE_HSV_LOW,   BLUE_HSV_HIGH)

    # ── Get wall vectors ─────────────────────────────────────────────────────
    ydx, ydy, y_count = _wall_vector(yellow_mask, cx, cy, CENTRE_DEAD_ZONE_PX)
    bdx, bdy, b_count = _wall_vector(blue_mask,   cx, cy, CENTRE_DEAD_ZONE_PX)

    have_yellow = y_count >= MIN_PIXELS
    have_blue   = b_count >= MIN_PIXELS
    have_both   = have_yellow and have_blue

    # Update stored wall directions whenever walls are clearly visible
    if have_yellow:
        _toward_yellow_x, _toward_yellow_y = _unit(ydx, ydy)
    if have_blue:
        _away_from_blue_x, _away_from_blue_y = _unit(-bdx, -bdy)

    # ── State transitions ────────────────────────────────────────────────────
    if have_both:
        _drive_state = 'both'
        _search_frames_left = 0
    elif have_yellow and not have_blue:
        if _drive_state in ('both', 'searching_yellow', 'corner_blue'):
            _drive_state = 'searching_blue'
            _search_frames_left = SEARCH_TIMEOUT_FRAMES
        elif _drive_state == 'searching_blue':
            _search_frames_left -= 1
            if _search_frames_left <= 0:
                _drive_state = 'corner_yellow'
    elif have_blue and not have_yellow:
        if _drive_state in ('both', 'searching_blue', 'corner_yellow'):
            _drive_state = 'searching_yellow'
            _search_frames_left = SEARCH_TIMEOUT_FRAMES
        elif _drive_state == 'searching_yellow':
            _search_frames_left -= 1
            if _search_frames_left <= 0:
                _drive_state = 'corner_blue'
    else:
        return _last_vx, _last_vy

    fwd_x = math.cos(heading)
    fwd_y = math.sin(heading)

    # ── Compute correction ───────────────────────────────────────────────────

    if _drive_state == 'both':
        # Target = midpoint between yellow and blue vectors from centre.
        # If midpoint is to the right, we're too close to yellow — push left.
        # The correction vector points FROM current position TOWARD midpoint.
        mid_dx = (ydx + bdx) / 2.0
        mid_dy = (ydy + bdy) / 2.0
        # Normalise midpoint offset by max radius
        corr_x = mid_dx / max_r
        corr_y = mid_dy / max_r
        raw_vx = fwd_x + corr_x * LATERAL_GAIN
        raw_vy = fwd_y + corr_y * LATERAL_GAIN

    elif _drive_state == 'searching_blue':
        # Have yellow, searching for blue. Maintain standoff from yellow
        # and nudge away from yellow (toward where blue should be).
        y_dist = math.hypot(ydx, ydy)
        target = SINGLE_WALL_TARGET_FRACTION * max_r
        # Positive error = yellow too far = we're too close to it
        standoff_error = (y_dist - target) / max_r
        # Push away from yellow (standoff) + nudge further away (search)
        away_x = -_toward_yellow_x if _toward_yellow_x is not None else 0.0
        away_y = -_toward_yellow_y if _toward_yellow_y is not None else 0.0
        raw_vx = fwd_x + away_x * (standoff_error * LATERAL_GAIN + SEARCH_NUDGE)
        raw_vy = fwd_y + away_y * (standoff_error * LATERAL_GAIN + SEARCH_NUDGE)

    elif _drive_state == 'searching_yellow':
        # Have blue, searching for yellow. Nudge away from blue.
        b_dist = math.hypot(bdx, bdy)
        target = SINGLE_WALL_TARGET_FRACTION * max_r
        standoff_error = (target - b_dist) / max_r
        away_x = _away_from_blue_x if _away_from_blue_x is not None else 0.0
        away_y = _away_from_blue_y if _away_from_blue_y is not None else 0.0
        raw_vx = fwd_x + away_x * (standoff_error * LATERAL_GAIN + SEARCH_NUDGE)
        raw_vy = fwd_y + away_y * (standoff_error * LATERAL_GAIN + SEARCH_NUDGE)

    elif _drive_state == 'corner_yellow':
        y_dist = math.hypot(ydx, ydy)
        target = SINGLE_WALL_TARGET_FRACTION * max_r
        standoff_error = (y_dist - target) / max_r
        away_x = -_toward_yellow_x if _toward_yellow_x is not None else 0.0
        away_y = -_toward_yellow_y if _toward_yellow_y is not None else 0.0
        raw_vx = fwd_x + away_x * standoff_error * LATERAL_GAIN
        raw_vy = fwd_y + away_y * standoff_error * LATERAL_GAIN

    elif _drive_state == 'corner_blue':
        b_dist = math.hypot(bdx, bdy)
        target = SINGLE_WALL_TARGET_FRACTION * max_r
        standoff_error = (target - b_dist) / max_r
        away_x = _away_from_blue_x if _away_from_blue_x is not None else 0.0
        away_y = _away_from_blue_y if _away_from_blue_y is not None else 0.0
        raw_vx = fwd_x + away_x * standoff_error * LATERAL_GAIN
        raw_vy = fwd_y + away_y * standoff_error * LATERAL_GAIN

    else:
        return _last_vx, _last_vy

    # ── Minimum forward component ─────────────────────────────────────────────
    fwd_component = raw_vx * fwd_x + raw_vy * fwd_y
    if fwd_component < MIN_FORWARD:
        raw_vx += fwd_x * (MIN_FORWARD - fwd_component)
        raw_vy += fwd_y * (MIN_FORWARD - fwd_component)

    vx, vy = _normalise(raw_vx, raw_vy, ROBOT_SPEED)
    _last_vx, _last_vy = vx, vy
    return vx, vy
