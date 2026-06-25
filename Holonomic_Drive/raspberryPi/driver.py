"""
driver.py — Autonomous lane-following + state machine driver for a
            front-wheel-drive, rear-wheel-steer chassis, fed by a real
            stitched 360-degree IPM camera image.

This is your state-machine code, adapted to run on real hardware instead
of the sim. Three kinds of changes were made, and nothing else:

  1. PREP FOR REAL CAMERA INPUT (preprocess_frame): the (W,H,3) transpose
     and RGB->BGR conversion were specific to pygame's array3d() output in
     the sim. A real OpenCV-based capture/stitch pipeline gives you (H,W,3)
     BGR natively, so that step is now a no-op by default (see
     INPUT_IS_BGR / PYGAME_WH_SWAPPED below).

  2. REMOVED WORLD-POSITION HEADING TRACKING (robot_heading): the old
     version recomputed "forward" every frame from world x,y deltas,
     because the sim's camera image was rendered world-axis-aligned. Real
     hardware has no ground-truth x,y. But the camera is rigidly bolted to
     the chassis, so "forward" in image-pixel-space never actually changes
     -- it's a fixed calibration constant, true for ANY drivetrain
     (holonomic or, as here, rear-steered), because the camera never moves
     relative to the body regardless of how the chassis points in the
     world. So robot_heading() now just sets a fixed bearing instead of
     tracking anything. See the ASSUMPTION note in that method -- it's the
     one thing here that depends on how your stitcher is built.

  3. REMOVED ARROW DETECTION, per your request: arrow_detection() is gone,
     along with the block in get_motion_command() that called it and the
     block in update_state() that let it force State.TURN_CHALLENGE. State
     machine and every compute_*_command() formula are otherwise untouched.

WHAT THIS MEANS FOR TURN_CHALLENGE
  TURN_CHALLENGE can still be entered the other way it always could --
  via the orientation-error check in update_state() (yellow/blue on the
  wrong side). But arrow_detection() used to be the only thing that set
  TURN_TIME to a positive duration; with it gone, that trigger would have
  fired and immediately expired (TURN_TIME starts at 0.0) every time. I
  added one constant, TURN_CHALLENGE_DURATION, set when the orientation
  trigger fires, so the state still actually runs for a beat instead of
  being dead code. It also still uses self.ARROW_DIRECTION to decide which
  way to turn -- nothing updates that anymore, so it'll always be whatever
  you set it to in __init__ ("Left" by default). Flag if you'd rather
  remove the orientation-triggered TURN_CHALLENGE entirely instead of
  giving it a fixed direction.

TWO PRE-EXISTING BUGS FIXED ALONG THE WAY (not related to the above, just
caught while reading through):
  - update_state(): `orientation` was only assigned inside the `else`
    branch of the ORIENATION_LOCK check, so referencing `orientation is
    False` further down relies on that branch having run. I checked
    whether this is reachable: ORIENATION_LOCK and POST_TURN_LOCK are
    always set to the same value at the same moment (only one place in the
    whole class sets either of them, and it sets both together) and
    decremented by the same dt each call, so they always cross zero on the
    same iteration -- POST_TURN_LOCK's early return currently always
    covers the case where orientation would be unbound. So this isn't an
    active crash today. But it's one assignment away from becoming one
    (e.g. if you ever add another path that resets ORIENATION_LOCK without
    also resetting POST_TURN_LOCK), so I added a default of None at the
    top of the function anyway -- zero behavior change, just removes a
    footgun that's currently dormant rather than live.
  - The old teleport-reset branch referenced self.drive_state, which
    doesn't exist anywhere else (everywhere else it's self.state) --
    moot now since that whole branch is gone with the heading rewrite,
    but flagging it since it was clearly a leftover from porting.
  - debug_data() builds the debug dict and stores it on self.dbg directly
    -- it doesn't return anything (implicit None). Both call sites did
    `self.dbg = self.debug_data(...)`, which immediately overwrote the
    dict debug_data() had just set with that None return value. So the
    debug_info you'd get back from get_motion_command() was always None,
    every single call, regardless of state. Fixed by calling
    `self.debug_data(...)` without the assignment at both call sites --
    debug_data() already owns setting self.dbg, so nothing else needs to
    capture its return value.

CALIBRATION YOU STILL NEED TO DO ON THE BENCH
  - FORWARD_BEARING_DEG (in __init__) -- only matters if your stitched
    image's pixel axes don't already line up with the chassis's physical
    forward/right. Leave at 0.0 if they do.
  - INPUT_IS_BGR / PYGAME_WH_SWAPPED -- match your real capture pipeline.
  - YELLOW_HSV_*, BLUE_HSV_* -- you're retuning these separately.
  - CENTRE_DEAD_ZONE_PX and friends are still raw pixel counts tuned to
    whatever resolution the sim happened to render at -- you'll want to
    re-check these once you know your real stitched resolution.
"""

import numpy as np
import math
import cv2
import time
from enum import Enum


class State(Enum):
    CENTER = 0 # Both lines visable
    SEARCH_YELLOW = 1 # Have blue, search for yellow
    CORNER_YELLOW = 2 
    SEARCH_BLUE = 3 # Have yellow, search for yellow
    CORNER_BLUE = 4
    OBJECT = 5 #Purple object detected
    TURN_CHALLENGE = 6 # orientation-error forced turn



class drive:
    def __init__(self):

        self.state = State.CENTER

        self.SPEED = 0.8
        
        # Minimum pixels to consider a wall visible
        self.MIN_PIXELS = 50

        # Dead zone: ignore pixels within this radius of centre (robot body)
        self.CENTRE_DEAD_ZONE_PX = 55

        # How strongly lateral error drives correction
        self.LATERAL_GAIN = 4 # from 2.5

        # Minimum forward component (fraction of ROBOT_SPEED)
        self.MIN_FORWARD = 0.2

        # Target distance from wall as fraction of max visible radius (single wall mode)
        self.SINGLE_WALL_TARGET_FRACTION = 0.45 # from 0.45

        # Frames to search for lost wall before committing to corner mode
        self.SEARCH_TIMEOUT = 3.0 # from 30fps

        # Nudge strength when searching for lost wall
        self.SEARCH_NUDGE = 1 # from 0.25

        self.ARROW_DIRECTION = "Left"

        self.TURN_TIME = 0.0

        # How long a TURN_CHALLENGE forced-turn runs once triggered.
        # arrow_detection() used to set TURN_TIME directly when it found an
        # arrow; with arrow detection removed, the orientation-error
        # trigger (in update_state) is now the only way into
        # TURN_CHALLENGE, so it needs its own duration to set TURN_TIME to.
        self.TURN_CHALLENGE_DURATION = 1.0

        self.ORIENATION_LOCK = 0
        self.CENTER_LOCK = 0

        self.search_time_left = 0

        self.POST_TURN_LOCK = 0

        self.STEER_SMOOTHING = 0.8  # 0..1 — lower = smoother/slower turn response

        self.TURN_LOCK =0.3 # previously 0.7 for day 1 prac tracks

        # -----------------------------------------------
        # Real-camera calibration (replaces the old dynamic_heading /
        # world x,y tracking -- see module docstring point 2)
        # -----------------------------------------------
        # Angle, in degrees, within the stitched IPM image's pixel
        # coordinates (x = column/right, y = row/down) that corresponds to
        # the chassis's physical forward direction. 0.0 means the image's
        # +x axis already IS forward and +y already IS right -- if your
        # stitcher is built that way, leave this alone and every formula
        # below works unchanged, since self.ydx/self.bdx etc. are then
        # already in the same frame as self.fwd_x/self.fwd_y.
        self.FORWARD_BEARING_DEG = 0.0

        # Most OpenCV-based capture/stitch pipelines hand you BGR uint8
        # frames natively. Set False if your stitcher gives RGB instead.
        self.INPUT_IS_BGR = True

        # Only relevant if you're still testing this file against the old
        # pygame sim, which returns arrays as (W,H,3). Leave False for real
        # camera input.
        self.PYGAME_WH_SWAPPED = False

        # -----------------------------------------------
        # Colour Params
        # -----------------------------------------------
        self.YELLOW_HSV_LOW  = np.array([ 20, 100, 100])
        self.YELLOW_HSV_HIGH = np.array([ 40, 255, 255])
        self.BLUE_HSV_LOW    = np.array([100, 100, 100])
        self.BLUE_HSV_HIGH   = np.array([130, 255, 255])
        # -----------------------------------------------
        #STATE?
        # -----------------------------------------------
        self.last_vx         = 0.0
        self.last_vy         = 0.0
        self.heading          = math.radians(self.FORWARD_BEARING_DEG)
        # No ground-truth world position on real hardware -- kept only so
        # debug_data()'s dict shape doesn't change.
        self.rx = 0.0
        self.ry = 0.0

        # Last known direction TO yellow wall and AWAY from blue wall (unit vectors)
        # These are stored in body frame and updated whenever walls are visible.
        # Used to maintain consistent push direction when only one wall is seen.
        self.toward_yellow_x = None
        self.toward_yellow_y = None
        self.away_from_blue_x = None
        self.away_from_blue_y = None

        self.last_time = None
    # -----------------------------------------------
    #HELPERS
    # -----------------------------------------------
    def wall_vector(self, mask, cx, cy, dead_zone):
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
        if count < self.MIN_PIXELS:
            return 0.0, 0.0, 0

        mean_dx = float(np.mean(dx))
        mean_dy = float(np.mean(dy))
        return mean_dx, mean_dy, count
    

    def unit(self, dx, dy):
        mag = math.hypot(dx, dy)
        if mag < 1e-6:
            return 0.0, 0.0
        return dx / mag, dy / mag


    def normalise(self, vx, vy, speed):
        mag = math.hypot(vx, vy)
        if mag < 1e-6:
            return 0.0, 0.0
        return vx / mag * speed, vy / mag * speed

    def line_side(self):
        self.forward_x, self.forward_y = 0, 1  # or your robot heading
        cross = self.forward_x * self.dy - self.forward_y *self.dx

        if cross > 0:
            return "left"
        elif cross < 0:
            return "right"
        else:
            return "center"
    

    # -----------------------------------------------
    #preprocess
    # -----------------------------------------------
    def preprocess_frame(self, camera_view_data):
        """
        CALIBRATE: INPUT_IS_BGR / PYGAME_WH_SWAPPED need to match your real
        capture/stitch pipeline. The old transpose(1,0,2) + RGB->BGR here
        were specific to pygame's array3d() output in the sim and would
        scramble a real (H,W,3) OpenCV frame.
        """
        frame = camera_view_data
        if self.PYGAME_WH_SWAPPED:
            frame = frame.transpose(1, 0, 2)

        if self.INPUT_IS_BGR:
            self.img_bgr = frame
        else:
            self.img_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        img_hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)

        img_h, img_w = img_hsv.shape[:2]
        self.cx, self.cy  = img_w / 2.0, img_h / 2.0

        # max distance from the image center to any corenr of the image
        self.max_r = math.hypot(self.cx, self.cy)


        self.yellow_mask = cv2.inRange(img_hsv,self.YELLOW_HSV_LOW, self.YELLOW_HSV_HIGH)
        self.blue_mask = cv2.inRange(img_hsv,self.BLUE_HSV_LOW, self.BLUE_HSV_HIGH)
    

    # -----------------------------------------------
    # wall detection
    # -----------------------------------------------
    def detect_lines(self):
        # ── Get line vectors ─────────────────────────────────────────────────────
        self.ydx, self.ydy, self.y_count = self.wall_vector(self.yellow_mask, self.cx, self.cy, self.CENTRE_DEAD_ZONE_PX)
        self.bdx, self.bdy, self.b_count = self.wall_vector(self.blue_mask,   self.cx, self.cy, self.CENTRE_DEAD_ZONE_PX)

        self.have_yellow = self.y_count >= self.MIN_PIXELS
        self.have_blue   = self.b_count >= self.MIN_PIXELS
        self.have_both   = self.have_yellow and self.have_blue

            # Update stored line directions whenever lines are clearly visible
        if self.have_yellow:
            self.toward_yellow_x, self.toward_yellow_y = self.unit(self.ydx, self.ydy)
        if self.have_blue:
            self.away_from_blue_x, self.away_from_blue_y = self.unit(-self.bdx, -self.bdy)


    def signed_side(self, dx, dy):
        return self.fwd_x * dy - self.fwd_y * dx
    
    def orientation_status(self):
        """
        True  = correctly oriented (yellow left / blue right, where visible)
        False = at least one visible wall is on the wrong side
        None  = not enough info to judge (neither wall visible)
        """
        yellow_left = None
        blue_right = None

        if self.have_yellow:
            yellow_left = self.signed_side(self.ydx, self.ydy) < 0

        if self.have_blue:
            blue_right = self.signed_side(self.bdx, self.bdy) > 0

        if yellow_left is None and blue_right is None:
            return None
        if yellow_left is False or blue_right is False:
            return False
        return True


    def robot_heading(self, robot_state):
        """
        Real hardware has no ground-truth world x,y, so there's nothing to
        dynamically track here the way the sim version did (recomputing
        "forward" every frame from world-position deltas). The camera
        array is rigidly mounted to the chassis, so "forward" in
        image-pixel-space is a fixed calibration constant -- it doesn't
        change as the chassis steers around the track, because the camera
        never moves relative to the body, regardless of drivetrain type.

        ASSUMPTION: your stitched IPM frame is built so the image's pixel
        x-axis already points along the robot's physical forward direction
        (and +y along physical right). With that true, FORWARD_BEARING_DEG
        = 0.0 is correct and nothing else in this file needs to change --
        self.ydx/self.bdx etc. are already in the same frame as
        self.fwd_x/self.fwd_y. If your stitcher's orientation differs,
        rotate the frame to match before calling this (e.g. in your bridge
        script) rather than touching the maths in here.
        """
        self.heading = math.radians(self.FORWARD_BEARING_DEG)
        # No real x,y on hardware -- kept only so debug_data()'s dict shape
        # doesn't change.
        self.rx, self.ry = 0.0, 0.0

    # -----------------------------------------------
    # debug stuff
    # -----------------------------------------------
    def debug_data(self, state_label):
        # ── Store debug frame for render_debug_panel ──────────────────────────────
        self.dbg = (self.img_bgr.copy(), {
            'have_yellow': self.have_yellow,
            'have_blue':   self.have_blue,
            'have_both':   self.have_both,
            'ydx': self.ydx, 'ydy': self.ydy, 'y_count': self.y_count,
            'bdx': self.bdx, 'bdy': self.bdy, 'b_count': self.b_count,
            'cmd_vx': self.vx,  'cmd_vy': self.vy,
            'state':   state_label,
            'heading': self.heading,
            'dead_zone': self.CENTRE_DEAD_ZONE_PX,
            'rx': self.rx, 'ry': self.ry,
        })


    # -----------------------------------------------
    # Main fucntions for self driving
    # -----------------------------------------------
    def finalize_velocity(self, raw_vx, raw_vy):
        """Apply the minimum-forward-component clamp and scale to SPEED."""
        fwd_component = raw_vx * self.fwd_x + raw_vy * self.fwd_y
        if fwd_component < self.MIN_FORWARD:
            raw_vx += self.fwd_x * (self.MIN_FORWARD - fwd_component)
            raw_vy += self.fwd_y * (self.MIN_FORWARD - fwd_component)
 
        return self.normalise(raw_vx, raw_vy, self.SPEED)
    
    def compute_center_command(self):
        mid_dx = (self.ydx + self.bdx) / 2.0
        mid_dy = (self.ydy + self.bdy) / 2.0

        corr_x = mid_dx / self.max_r
        corr_y = mid_dy / self.max_r

        raw_vx = self.fwd_x + corr_x * self.LATERAL_GAIN
        raw_vy = self.fwd_y + corr_y * self.LATERAL_GAIN

        return self.finalize_velocity(raw_vx, raw_vy)
        

    def compute_yellow_bias_command(self, searching):

        y_dist = math.hypot(self.ydx, self.ydy)
        target = self.SINGLE_WALL_TARGET_FRACTION * self.max_r
        standoff_error = (target - y_dist) / self.max_r

        away_x = -self.toward_yellow_x if self.toward_yellow_x is not None else 0.0
        away_y = -self.toward_yellow_y if self.toward_yellow_y is not None else 0.0

        raw_vx = self.fwd_x + away_x * (standoff_error * self.LATERAL_GAIN + self.SEARCH_NUDGE)
        raw_vy = self.fwd_y + away_y * (standoff_error * self.LATERAL_GAIN + self.SEARCH_NUDGE)

        gain = standoff_error * self.LATERAL_GAIN
        if searching:
            gain += self.SEARCH_NUDGE
 
        raw_vx = self.fwd_x + away_x * gain
        raw_vy = self.fwd_y + away_y * gain

        return self.finalize_velocity(raw_vx, raw_vy)

    def compute_blue_bias_command(self, searching):
        b_dist = math.hypot(self.bdx, self.bdy)
        target = self.SINGLE_WALL_TARGET_FRACTION * self.max_r
        standoff_error = (target - b_dist) / self.max_r

        away_x = self.away_from_blue_x if self.away_from_blue_x is not None else 0.0
        away_y = self.away_from_blue_y if self.away_from_blue_y is not None else 0.0

        raw_vx = self.fwd_x + away_x * (standoff_error * self.LATERAL_GAIN + self.SEARCH_NUDGE)
        raw_vy = self.fwd_y + away_y * (standoff_error * self.LATERAL_GAIN + self.SEARCH_NUDGE)

        gain = standoff_error * self.LATERAL_GAIN
        if searching:
            gain += self.SEARCH_NUDGE
 
        raw_vx = self.fwd_x + away_x * gain
        raw_vy = self.fwd_y + away_y * gain
        return self.finalize_velocity(raw_vx, raw_vy)
    
    def compute_turn_command(self, direction):
        """
        Turn challenge command, ignoring the colour and forcing the kiwi into one of two dirctions
        """
        normal_x, normal_y = self.fwd_y, -self.fwd_x # computes the left perpendicular vector
        if direction == 'right':
            normal_x, normal_y = -normal_x, -normal_y

        TURN_SIDE = 0.4
        TURN_FWD = 1.2

        #raw_vx = self.fwd_x + normal_x * self.SEARCH_NUDGE
        #raw_vy = self.fwd_y + normal_y * self.SEARCH_NUDGE
        raw_vx = self.fwd_x * TURN_FWD + normal_x * TURN_SIDE
        raw_vy = self.fwd_y * TURN_FWD + normal_y * TURN_SIDE

        return self.finalize_velocity(raw_vx, raw_vy)
    


    # -----------------------------------------------
    #Updating State machine
    # -----------------------------------------------
    def update_state(self):

        """
        This function 
        """
        # `orientation` must default to something even when the lock skips
        # computing it below -- previously unset in that case, which would
        # raise UnboundLocalError on the `orientation is False` check
        # further down on any frame right after a turn (when the lock is
        # still active). None matches orientation_status()'s own
        # "not enough info" return value.
        orientation = None

        # ---orientation lock---
        if self.ORIENATION_LOCK > 0:
            self.ORIENATION_LOCK -= self.dt
        else:
            orientation = self.orientation_status()

        if self.CENTER_LOCK > 0:
            self.CENTER_LOCK -= self.dt

        # --post-turn lock---
        if self.POST_TURN_LOCK > 0:
            self.POST_TURN_LOCK -= self.dt
            return


        # ---TURN CHALLENGE---
        if self.state == State.TURN_CHALLENGE:
            self.TURN_TIME -= self.dt
            if self.TURN_TIME <= 0:
                self.ORIENATION_LOCK = self.TURN_LOCK
                self.CENTER_LOCK = self.TURN_LOCK
                self.POST_TURN_LOCK = self.TURN_LOCK 

                if self.ARROW_DIRECTION == "Left":
                    self.state = State.SEARCH_BLUE
                else:
                    self.state = State.SEARCH_YELLOW

                self.search_time_left = self.SEARCH_TIMEOUT
            return

        # ---orientation error triggers turn---
        if orientation is False and self.ORIENATION_LOCK <= 0:
            self.state = State.TURN_CHALLENGE
            self.TURN_TIME = self.TURN_CHALLENGE_DURATION
            return

        # ---CENTER---
        if self.have_both and self.CENTER_LOCK <= 0:
            self.state = State.CENTER
            self.search_time_left = 0
            return

        # ---SEARCH / CORNER logic---
        if self.have_yellow:
            if self.state in (State.CENTER, State.SEARCH_YELLOW, State.CORNER_BLUE):
                self.state = State.SEARCH_BLUE
                self.search_time_left = self.SEARCH_TIMEOUT
            elif self.state == State.SEARCH_BLUE:
                self.search_time_left -= self.dt
                if self.search_time_left <= 0:
                    self.state = State.CORNER_YELLOW

        else:
            if self.state in (State.CENTER, State.SEARCH_BLUE, State.CORNER_YELLOW):
                self.state = State.SEARCH_YELLOW
                self.search_time_left = self.SEARCH_TIMEOUT
            elif self.state == State.SEARCH_YELLOW:
                self.search_time_left -= self.dt
                if self.search_time_left <= 0:
                    self.state = State.CORNER_BLUE

    # -----------------------------------------------
    #State machine
    # -----------------------------------------------
    def get_motion_command(self, robot_state, camera_view_data):

        now = time.time()
        if self.last_time is None:
            self.dt = 1/15.0   # assume 15fps first frame
        else:
            self.dt = now - self.last_time
        self.last_time = now

        self.robot_heading(robot_state)

        self.preprocess_frame(camera_view_data)
        self.detect_lines()

        self.fwd_x = math.cos(self.heading)
        self.fwd_y = math.sin(self.heading)

        if not self.have_yellow and not self.have_blue:
            # No lines at all: hold the last command and deliberately leave
            # self.state / self.search_frames_left untouched, so a brief
            # total dropout doesn't reset or advance the search timer.
            self.vx, self.vy = self.last_vx, self.last_vy
            self.debug_data('no_walls')
            return self.vx, self.vy, self.dbg

        self.update_state()

        print("State: ", self.state.name)

        match self.state:
            case State.CENTER:
                vx, vy = self.compute_center_command()
            case State.SEARCH_BLUE:
                vx, vy = self.compute_yellow_bias_command(searching=True)
            case State.CORNER_YELLOW:
                vx, vy = self.compute_yellow_bias_command(searching=False)
            case State.SEARCH_YELLOW:
                vx, vy = self.compute_blue_bias_command(searching=True)
            case State.CORNER_BLUE:
                vx, vy = self.compute_blue_bias_command(searching=False)
            case State.OBJECT:
                # NOTE: compute_object_command() doesn't exist yet, and
                # nothing in update_state() ever actually sets
                # State.OBJECT, so this is unreachable for now rather than
                # a live bug. Left as-is since it's unrelated to what was
                # asked for here -- flagging in case you wire this up later.
                vx, vy = self.compute_object_command()
            case State.TURN_CHALLENGE:
                if self.ARROW_DIRECTION == "Right":
                    vx, vy = self.compute_turn_command('right')
                elif self.ARROW_DIRECTION == "Left":
                    vx, vy = self.compute_turn_command('left')


        blend_x = self.last_vx + (vx - self.last_vx) * self.STEER_SMOOTHING
        blend_y = self.last_vy + (vy - self.last_vy) * self.STEER_SMOOTHING

        if math.hypot(blend_x, blend_y) < 1e-3:
            pass
        else:
            vx, vy = self.normalise(blend_x, blend_y, self.SPEED)
        
        self.vx, self.vy = vx, vy
        self.last_vx, self.last_vy = vx, vy
        self.debug_data(self.state.name.lower())
        return self.vx, self.vy, self.dbg



_driver = drive()
 
 
def get_motion_command(robot_state, camera_view_data):
    return _driver.get_motion_command(robot_state, camera_view_data)