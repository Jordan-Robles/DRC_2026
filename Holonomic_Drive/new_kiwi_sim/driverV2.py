
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
    TURN_CHALLENGE = 6 # arrow detection
    



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

        # Dynamic heading tracking rate
        self.HEADING_TRACK_RATE = 0.15

        # Frames to search for lost wall before committing to corner mode
        self.SEARCH_TIMEOUT = 3.0 # from 30fps

        # Nudge strength when searching for lost wall
        self.SEARCH_NUDGE = 1 # from 0.25

        self.ARROW_DIRECTION = "Left"

        self.ARROW_ACTIVE = False

        self.MIN_ARROW_AREA = 5000 

        self.TURN_TIME = 0.0

        self.ORIENATION_LOCK = 0
        self.CENTER_LOCK = 0

        self.search_time_left = 0

        self.POST_TURN_LOCK = 0

        self.STEER_SMOOTHING = 0.8  # 0..1 — lower = smoother/slower turn response



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
        self.dynamic_heading = None
        self.last_x          = None
        self.last_y          = None

        # Last known direction TO yellow wall and AWAY from blue wall (unit vectors)
        # These are stored in world frame and updated whenever walls are visible.
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
        img_rgb = camera_view_data.transpose(1, 0, 2)
        self.img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
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
    

    def arrow_detection(self):
        """
        Arrow detection through comparignt eh centorid of the arrow against the robots heading
        """
        img_gray = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2GRAY)
        _, gray_mask = cv2.threshold(img_gray, 80, 255, cv2.THRESH_BINARY_INV)

        canny_image = cv2.Canny(gray_mask, 50, 100)
        contours,_ = cv2.findContours(canny_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return None
        
        arrow_contour = max(contours, key = cv2.contourArea)

        #if cv2.contourArea(arrow_contour) < self.MIN_ARROW_AREA:
            #return None

        M = cv2.moments(arrow_contour)
        #The m00 notation indicates the sum of all coordiante of the contour of pxiels
        if M["m00"] == 0:
            return None
        
        #relation for the x/y coordinate of the centroid was dervied from openCV
        arrow_cx = int(M["m10"] / M["m00"])
        arrow_cy = int(M["m01"] / M["m00"])

        # Require arrow to be close enough vertically (distance proxy)
        # Arrow centroid must be in the lower 35% of the image (closer to robot)
        if arrow_cy < self.cy * 0.65:
            return None

        
        #We find the fartehs point form the centroid which is the arrow tip
        max_dist = -1
        tip = None
        #we reshape the return of the contour so we can retrieve a clean 2 element array
        for p in arrow_contour.reshape(-1,2):
            dx = p[0] - arrow_cx
            dy = p[1] - arrow_cy
            dist = dx*dx + dy*dy
            if dist > max_dist:
                max_dist = dist
                tip = (p[0], p[1])
                
        if tip is None:
            return None
        
        dx = tip[0] - arrow_cx
        dy = tip[1] - arrow_cy
        if dy < 0:
            return None
        
        if dx < 0:
            self.ARROW_DIRECTION = "Left"
        else:
            self.ARROW_DIRECTION = "Right"

        print(f"dx={dx} dy={dy} -> {self.ARROW_DIRECTION}")

        return self.ARROW_DIRECTION


    def robot_heading(self, robot_state):
        manual_heading = robot_state.get('heading', 0.0)
        self.rx = robot_state.get('x', 0.0)
        self.ry = robot_state.get('y', 0.0)

        # ── Dynamic heading ──────────────────────────────────────────────────────
        if self.dynamic_heading is None:
            self.dynamic_heading = manual_heading

        if self.last_x is not None:
            ddx = self.rx - self.last_x
            ddy = self.ry - self.last_y
            dist = math.hypot(ddx, ddy)
            if dist > 0.5:
                # Reset
                self.dynamic_heading    = manual_heading
                self.drive_state        = State.CENTER
                self.search_time_left = 0
                self.toward_yellow_x    = None
                self.toward_yellow_y    = None
                self.away_from_blue_x   = None
                self.away_from_blue_y   = None
                self.last_x, self.last_y    =self.rx, self.ry
                return 0.0, 0.0, None
            if dist > 1e-4:
                travel = math.atan2(ddy, ddx)
                diff = (travel - self.dynamic_heading + math.pi) % (2 * math.pi) - math.pi
                self.dynamic_heading = (self.dynamic_heading + self.HEADING_TRACK_RATE * diff) % (2 * math.pi)

        self.last_x, self.last_y = self.rx, self.ry
        self.heading = self.dynamic_heading

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
                self.ARROW_ACTIVE = False
                self.ORIENATION_LOCK = 0.7
                self.CENTER_LOCK = 0.7
                self.POST_TURN_LOCK = 0.7 

                if self.ARROW_DIRECTION == "Left":
                    self.state = State.SEARCH_BLUE
                else:
                    self.state = State.SEARCH_YELLOW

                self.search_time_left = self.SEARCH_TIMEOUT
            return

        # ---arrow triggers turn---
        if self.ARROW_ACTIVE and self.state != State.TURN_CHALLENGE:
            self.state = State.TURN_CHALLENGE
            return

        # ---orientation error triggers turn---
        if orientation is False and self.ORIENATION_LOCK <= 0:
            self.state = State.TURN_CHALLENGE
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

       
        if self.robot_heading(robot_state):
            # Robot teleported/reset this frame - matches driver.py's
            # early `return 0.0, 0.0, None` before any wall detection runs.
            self.last_vx, self.last_vy = 0.0, 0.0
            return 0.0, 0.0, None

        self.preprocess_frame(camera_view_data)
        self.detect_lines()

        self.fwd_x = math.cos(self.heading)
        self.fwd_y = math.sin(self.heading)

        # arrrow detection
        if not self.ARROW_ACTIVE:
            arrow_dir = self.arrow_detection()
            if arrow_dir is not None:
                self.ARROW_DIRECTION = arrow_dir
                self.ARROW_ACTIVE = True
                self.TURN_TIME = 1.0



        
        if not self.have_yellow and not self.have_blue:
            # No lines at all: hold the last command and deliberately leave
            # self.state / self.search_frames_left untouched, so a brief
            # total dropout doesn't reset or advance the search timer.
            self.vx, self.vy = self.last_vx, self.last_vy
            self.dbg = self.debug_data('no_walls')
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
        self.dbg = self.debug_data(self.state.name.lower())
        return self.vx, self.vy, self.dbg



_driver = drive()
 
 
def get_motion_command(robot_state, camera_view_data):
    return _driver.get_motion_command(robot_state, camera_view_data)
