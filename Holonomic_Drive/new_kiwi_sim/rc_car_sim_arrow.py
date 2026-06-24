"""
Autonomous RC Car Simulator — Top-Down 2D Pygame Simulation
Kiwi (omni) drive, locked-rotation (body never yaws), 6-camera hexagonal array.

On launch a file-picker popup lists every .py file in the same folder as this
script.  Select a track exported from the DRC Track Builder and click Load.

Controls:
  WASD  — translate robot
  ESC   — quit
"""

import tkinter as tk
from tkinter import messagebox, filedialog
import numpy as np
import math
import sys
import os
import importlib.util

# pygame imported AFTER tkinter is fully initialised — required on macOS
# to avoid the SDL/Tk event-loop conflict that causes a black screen.
import pygame

# ─────────────────────────────────────────────────────────────────────────────
# DISPLAY / ROBOT CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
TOP_PX         = 700          # top-down view width  (px)
TOP_PY         = 800          # top-down view height (px)
CAM_WIN_SIZE   = 600          # IPM view (square, px)
FPS            = 60
ROBOT_SPEED    = 0.5          # m/s at full key deflection

ROBOT_RADIUS_M = 0.090        # body radius for drawing (m)
CAM_ARRAY_R_M  = 0.080        # hex camera array radius (m)
CAM_HFOV_DEG   = 50.0         # per-camera horizontal FOV
CAM_NEAR_M     = 0.242        # near sensing distance (m)
CAM_FAR_M      = 0.600        # far sensing distance / IPM cap (m)
NUM_CAMS       = 6

TAPE_W_M       = 0.036        # tape width (m)
TRACK_W_M      = 1.07         # full track corridor width (m)

ARROW_ICON_SIZE_M   = 0.6     # on-track size of a direction-arrow marker (m) —
                               # matches the DRC Track Builder's 2x2-cell block
                               # (0.5m/cell * 2 cells * 0.6 scale-down = 0.6m)
ARROW_DETECT_RANGE_M = CAM_FAR_M  # radial range within which an arrow is "visible" to the sensor array

# Colours
C_YELLOW   = (230, 200,  20)
C_BLUE     = ( 40, 110, 235)
C_BG       = (225, 225, 228)
C_FLOOR    = (225, 225, 228)
C_GRID     = (200, 200, 205)
C_ROB_FILL = ( 55,  55,  65)
C_ROB_RING = ( 45,  45,  55)
C_NOSE     = (255,  75,  75)
C_WEDGE    = (206, 209, 222)
C_WEDGE_BD = (150, 155, 185)
C_CAM_DOT  = ( 90,  90, 200)
C_GREEN    = (  0, 224,  96)
C_ARROW    = ( 10,  10,  10)


# ─────────────────────────────────────────────────────────────────────────────
# GEOMETRY HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def arc_pts(cx, cy, r, a0_deg, a1_deg, n=20):
    """Polyline points along a CCW arc from a0_deg to a1_deg."""
    a0, a1 = math.radians(a0_deg), math.radians(a1_deg)
    if a1 <= a0:
        a1 += 2 * math.pi
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / n),
             cy + r * math.sin(a0 + (a1 - a0) * i / n))
            for i in range(n + 1)]


def pts_to_segs(pts):
    """Convert ordered point list → list of ((x0,y0),(x1,y1)) segments."""
    return [((pts[i][0], pts[i][1]), (pts[i+1][0], pts[i+1][1]))
            for i in range(len(pts) - 1)]


def rounded_rect_pts(xl, xr, yb, yt, r, n=20):
    """CCW closed polygon of a rounded rectangle."""
    if r <= 0:
        pts = [(xl, yb), (xr, yb), (xr, yt), (xl, yt), (xl, yb)]
        return pts
    pts  = arc_pts(xr - r, yb + r, r, -90,   0, n)
    pts += arc_pts(xr - r, yt - r, r,   0,  90, n)
    pts += arc_pts(xl + r, yt - r, r,  90, 180, n)
    pts += arc_pts(xl + r, yb + r, r, 180, 270, n)
    pts.append(pts[0])
    return pts


def arrow_icon_points(ax, ay, direction, size=ARROW_ICON_SIZE_M):
    """
    World-space geometry for a direction-arrow marker: a straight stem and a
    triangular arrowhead perpendicular to it at one end — mirrors the DRC
    Track Builder's on-canvas icon (no curves), in metres instead of pixels.
    direction: 'L' / 'R' / 'U' / 'D'
    Returns (p_top, p_bot, tip, base1, base2) world points.
    """
    stem_near = -size * 0.28
    stem_far  =  size * 0.16
    head_len  =  size * 0.34
    head_half =  size * 0.22

    if direction in ("L", "R"):
        d = 1.0 if direction == "R" else -1.0
        p_top = (ax, ay + stem_near)
        p_bot = (ax, ay + stem_far)
        tip   = (ax + d * head_len, ay + stem_far)
        base1 = (ax, ay + stem_far - head_half)
        base2 = (ax, ay + stem_far + head_half)
    else:  # 'U' / 'D'
        d = 1.0 if direction == "D" else -1.0
        p_top = (ax + stem_near, ay)
        p_bot = (ax + stem_far,  ay)
        tip   = (ax + stem_far, ay + d * head_len)
        base1 = (ax + stem_far - head_half, ay)
        base2 = (ax + stem_far + head_half, ay)

    return p_top, p_bot, tip, base1, base2


def draw_arrow_marker(surface, ax, ay, direction, to_screen, line_px, colour=C_ARROW,
                       size=ARROW_ICON_SIZE_M):
    """
    Draw one arrow marker directly: a stroked stem + a FILLED arrowhead
    triangle (not three separate stroked edges — stroking each edge of a
    small triangle makes the strokes overlap into an unrecognisable blob).
    to_screen(x, y) maps a world point to a screen-pixel tuple.
    """
    p_top, p_bot, tip, base1, base2 = arrow_icon_points(ax, ay, direction, size)
    pygame.draw.line(surface, colour, to_screen(*p_top), to_screen(*p_bot), line_px)
    pygame.draw.polygon(surface, colour,
                         [to_screen(*tip), to_screen(*base1), to_screen(*base2)])


def wedge_poly(apex_x, apex_y, direction_rad, half_fov, near, far, n=14):
    """
    Convex polygon for a truncated angular wedge (camera footprint).
    near-arc (left→right) then far-arc (right→left).
    Always returned in CCW winding so clip_seg_to_convex_poly works correctly.
    """
    pts = []
    for k in range(n + 1):
        a = direction_rad - half_fov + 2 * half_fov * k / n
        pts.append((apex_x + near * math.cos(a), apex_y + near * math.sin(a)))
    for k in range(n, -1, -1):
        a = direction_rad - half_fov + 2 * half_fov * k / n
        pts.append((apex_x + far * math.cos(a),  apex_y + far * math.sin(a)))
    # Ensure CCW winding (shoelace signed area > 0)
    nn = len(pts)
    area2 = sum(pts[i][0] * pts[(i+1) % nn][1] - pts[(i+1) % nn][0] * pts[i][1]
                for i in range(nn))
    if area2 < 0:
        pts = pts[::-1]
    return pts


def clip_seg_to_convex_poly(p1, p2, poly):
    """
    Parametric clip of line segment p1→p2 against a CCW convex polygon.
    Tracks the entering/leaving t values along the segment.
    Returns clipped (q1, q2) or None if entirely outside.
    """
    px, py = p1
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    t_in, t_out = 0.0, 1.0
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        # Inward normal for CCW edge A→B = rotate (B-A) left 90°
        nx, ny   = -(by - ay), (bx - ax)
        # Signed distance of P1 from this edge (positive = inside)
        numer_p1 = nx * (px - ax) + ny * (py - ay)
        denom    = nx * dx + ny * dy
        if abs(denom) < 1e-12:
            if numer_p1 < -1e-9:
                return None          # parallel and outside
        else:
            t = -numer_p1 / denom   # parameter where segment crosses edge line
            if denom < 0:           # moving away → sets exit bound
                t_out = min(t_out, t)
            else:                   # moving toward → sets entry bound
                t_in  = max(t_in, t)
        if t_in > t_out + 1e-9:
            return None
    if t_in > t_out + 1e-9:
        return None
    return ((px + t_in  * dx, py + t_in  * dy),
            (px + t_out * dx, py + t_out * dy))


# ─────────────────────────────────────────────────────────────────────────────
# MAP DEFINITIONS
# ─────────────────────────────────────────────────────────────────────────────

def map_oval():
    """Classic oval — rounded rectangle, 7×8 m arena."""
    AW, AH = 7.0, 8.0
    XL, XR, YB, YT = 0.70, 6.30, 0.60, 7.40
    R_OUT = 1.00
    R_IN  = max(0.05, R_OUT - TRACK_W_M)
    outer = pts_to_segs(rounded_rect_pts(XL,            XR,            YB,            YT,            R_OUT))
    inner = pts_to_segs(rounded_rect_pts(XL + TRACK_W_M, XR - TRACK_W_M, YB + TRACK_W_M, YT - TRACK_W_M, R_IN))
    return outer, inner, 3.5, 4.0, AW, AH


def map_figure8():
    """
    Figure-8: two tangent circular loops, 8×9 m arena.
    Outer boundary = two large circles; inner = two smaller circles.
    The crossover region is left open (no inner tape) so the robot
    can drive through the centre.
    """
    AW, AH  = 8.0, 9.0
    CX      = 4.0
    CY_LO   = 2.4
    CY_HI   = 6.6
    R_OUT   = 2.0
    R_IN    = R_OUT - TRACK_W_M

    def circle_segs(cx, cy, r, n=48):
        pts = arc_pts(cx, cy, r, 0, 360, n)
        pts.append(pts[0])
        return pts_to_segs(pts)

    outer = circle_segs(CX, CY_LO, R_OUT) + circle_segs(CX, CY_HI, R_OUT)
    inner = circle_segs(CX, CY_LO, R_IN)  + circle_segs(CX, CY_HI, R_IN)
    return outer, inner, CX, CY_LO + 0.5, AW, AH


def map_hairpin():
    """Narrow hairpin loop forcing tight turns, 7×8 m arena."""
    AW, AH = 7.0, 8.0
    XL, XR, YB, YT = 1.80, 5.20, 0.80, 7.20
    R_OUT = 0.60
    R_IN  = max(0.02, R_OUT - TRACK_W_M)
    outer = pts_to_segs(rounded_rect_pts(XL,            XR,            YB,            YT,            R_OUT))
    inner = pts_to_segs(rounded_rect_pts(XL + TRACK_W_M, XR - TRACK_W_M, YB + TRACK_W_M, YT - TRACK_W_M, R_IN))
    return outer, inner, 3.5, 4.0, AW, AH


# ─────────────────────────────────────────────────────────────────────────────
# TRACK FILE LOADING
# ─────────────────────────────────────────────────────────────────────────────


def load_map_file(path):
    """
    Load a track.py exported by the DRC Track Builder.
    Coordinates are Y-down (origin top-left) matching the builder directly —
    no Y-flip needed. The sim viewport handles Y-down rendering.
    """
    spec   = importlib.util.spec_from_file_location("_track", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    aw = float(getattr(module, 'ARENA_W', 12.0))
    ah = float(getattr(module, 'ARENA_H', 10.0))
    sx = float(getattr(module, 'SPAWN_X', aw / 2))
    sy = float(getattr(module, 'SPAWN_Y', ah / 2))

    def norm(segs):
        return [((float(s[0][0]), float(s[0][1])),
                 (float(s[1][0]), float(s[1][1]))) for s in segs]

    outer = norm(getattr(module, 'YELLOW_SEGMENTS', []))
    inner = norm(getattr(module, 'BLUE_SEGMENTS',   []))
    green = norm(getattr(module, 'GREEN_SEGMENTS',  []))

    # Direction-arrow markers: (x, y, direction) tuples, direction in L/R/U/D
    raw_arrows = getattr(module, 'ARROW_MARKERS', [])
    arrows = []
    for a in raw_arrows:
        ax, ay, adir = float(a[0]), float(a[1]), str(a[2]).upper()
        if adir not in ("L", "R", "U", "D"):
            continue
        arrows.append({'x': ax, 'y': ay, 'dir': adir})

    # Pre-build the world-space icon geometry once (arrows are static) so it
    # can be rendered with exactly the same line-clipping path used for tape.
    # Scale so the track fills the view width; clamp to a readable min
    m2px      = max(20.0, TOP_PX / aw)
    tape_px   = max(2, int(TAPE_W_M * m2px))
    arrow_px  = max(2, int(TAPE_W_M * 1.3 * m2px))
    return dict(outer=outer, inner=inner, green=green,
                arrows=arrows,
                spawn=(sx, sy),
                arena_w=aw, arena_h=ah, m2px=m2px,
                tape_px=tape_px, arrow_px=arrow_px, name=os.path.basename(path))


# ─────────────────────────────────────────────────────────────────────────────
# TRACK PICKER  (runs before pygame, uses tkinter)
# ─────────────────────────────────────────────────────────────────────────────

def pick_track_file():
    """
    Open a native file-chooser dialog starting in the script's own folder,
    filtered to .py files.  Returns the chosen path or None if cancelled.
    """
    import subprocess, platform
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # On macOS, use osascript to bring Python to the front BEFORE tkinter opens.
    # This is the only reliable way to get the dialog above other apps.
    if platform.system() == "Darwin":
        try:
            subprocess.run(
                ["osascript", "-e",
                 'tell application "System Events" to set frontmost of '
                 '(first process whose unix id is {}) to true'.format(os.getpid())],
                check=False, capture_output=True, timeout=2
            )
        except Exception:
            pass

    root = tk.Tk()
    root.withdraw()
    root.lift()
    root.focus_force()
    root.attributes('-topmost', True)
    path = filedialog.askopenfilename(
        title="Select track file",
        initialdir=script_dir,
        filetypes=[("Python track files", "*.py"), ("All files", "*.*")],
        parent=root,
    )
    root.destroy()
    return path if path else None


# ─────────────────────────────────────────────────────────────────────────────
# COORDINATE UTILITIES
# ─────────────────────────────────────────────────────────────────────────────

def w2s(x_m, y_m, m2px, pan_x, pan_y):
    """World metres (Y-down) → top-down screen pixels with pan offset."""
    return int(x_m * m2px - pan_x), int(y_m * m2px - pan_y)


# Camera IPM view: centred on robot, fixed scale
CAM_HALF_M = CAM_FAR_M + 0.12           # half-width of IPM view in metres
CAM_SCALE  = (CAM_WIN_SIZE / 2) / CAM_HALF_M   # px / m

def r2c(dx_m, dy_m):
    """Robot-relative world offset (m) → IPM screen pixel (Y-down world)."""
    return (int(CAM_WIN_SIZE / 2 + dx_m * CAM_SCALE),
            int(CAM_WIN_SIZE / 2 + dy_m * CAM_SCALE))


# ─────────────────────────────────────────────────────────────────────────────
# ARROW DETECTION  (which markers are within sensing range right now)
# ─────────────────────────────────────────────────────────────────────────────

def get_visible_arrows(rx, ry, map_data, max_range=ARROW_DETECT_RANGE_M):
    """
    Returns the direction-arrow markers currently within the sensor array's
    radial range, nearest first, with robot-relative geometry attached so
    the driver doesn't have to recompute it:
      x, y         — world position (m)
      direction    — 'L' / 'R' / 'U' / 'D'
      dx, dy       — offset from robot centre, world/robot frame (m)
                      (body never yaws, so world frame == robot frame)
      distance     — radial distance from robot centre (m)
      bearing      — atan2(dy, dx) in radians, world/robot frame
    """
    visible = []
    for a in map_data.get('arrows', []):
        dx = a['x'] - rx
        dy = a['y'] - ry
        dist = math.hypot(dx, dy)
        if dist <= max_range:
            visible.append({
                'x': a['x'], 'y': a['y'], 'direction': a['dir'],
                'dx': dx, 'dy': dy,
                'distance': dist,
                'bearing': math.atan2(dy, dx),
            })
    visible.sort(key=lambda v: v['distance'])
    return visible


# ─────────────────────────────────────────────────────────────────────────────
# CAMERA IPM RENDERING
# ─────────────────────────────────────────────────────────────────────────────

def render_camera_view(surface, rx, ry, map_data, motion_angle=0.0, debug_frame=None):
    """
    Render the stitched 360° IPM floor map, and overlay driver decision vectors
    if debug_frame is provided (AUTO mode).
    Returns a (W, H, 3) numpy array of the rendered surface.
    motion_angle: radians, world-frame direction of current velocity command.
                  The red nose dot is drawn at this angle so it tracks actual
                  travel direction in both teleop and autonomous modes.
    debug_frame: (img_bgr, info) tuple from driver.get_motion_command, or None.
    """
    surface.fill(C_FLOOR)
    half_fov = math.radians(CAM_HFOV_DEG) / 2.0
    full_fov = half_fov * 2
    tape_px  = max(2, int(TAPE_W_M * CAM_SCALE))
    arrow_px = max(2, int(TAPE_W_M * 1.3 * CAM_SCALE))

    # Scale rings
    for r_m in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6):
        pygame.draw.circle(surface, (32, 32, 42),
                           (CAM_WIN_SIZE // 2, CAM_WIN_SIZE // 2),
                           int(r_m * CAM_SCALE), 1)

    # Per-camera wedge shading
    for i in range(NUM_CAMS):
        cam_dir = math.radians(i * 60)
        fx = rx + CAM_ARRAY_R_M * math.cos(cam_dir)  # camera face world pos
        fy = ry + CAM_ARRAY_R_M * math.sin(cam_dir)
        fdx, fdy = fx - rx, fy - ry                  # face offset from robot

        # Build wedge polygon in robot-relative IPM screen coords
        pts = []
        for k in range(15):
            a = cam_dir - half_fov + full_fov * k / 14
            pts.append(r2c(fdx + CAM_NEAR_M * math.cos(a),
                           fdy + CAM_NEAR_M * math.sin(a)))
        for k in range(14, -1, -1):
            a = cam_dir - half_fov + full_fov * k / 14
            pts.append(r2c(fdx + CAM_FAR_M * math.cos(a),
                           fdy + CAM_FAR_M * math.sin(a)))

        if len(pts) >= 3:
            pygame.draw.polygon(surface, C_WEDGE, pts)
            pygame.draw.polygon(surface, C_WEDGE_BD, pts, 1)

    # Tape lines — yellow and blue first across ALL cameras, then green on top.
    # Green must be a completely separate pass AFTER all blue is done, otherwise
    # a later camera's blue wedge can overdraw an earlier camera's green pixels.
    tape_px_green = tape_px

    def draw_layer(segs, colour, lpx):
        for (p1, p2) in segs:
            for i in range(NUM_CAMS):
                cam_dir = math.radians(i * 60)
                fx = rx + CAM_ARRAY_R_M * math.cos(cam_dir)
                fy = ry + CAM_ARRAY_R_M * math.sin(cam_dir)
                wpoly = wedge_poly(fx, fy, cam_dir, half_fov, CAM_NEAR_M, CAM_FAR_M)
                result = clip_seg_to_convex_poly(p1, p2, wpoly)
                if result is None:
                    continue
                q1, q2 = result
                sp1 = r2c(q1[0] - rx, q1[1] - ry)
                sp2 = r2c(q2[0] - rx, q2[1] - ry)
                pygame.draw.line(surface, colour, sp1, sp2, lpx)

    draw_layer(map_data['outer'],        C_YELLOW, tape_px)
    draw_layer(map_data['inner'],        C_BLUE,   tape_px)
    # Green drawn in its own complete pass — guaranteed on top of everything
    draw_layer(map_data.get('green',[]), C_GREEN,  tape_px_green)
    # Arrow markers — drawn directly (stem stroke + filled head triangle)
    # rather than via the per-wedge line clipper above: clipping a tiny
    # triangle's edges into camera-wedge fragments is what produced the
    # overlapping-blob look. Drawn last so they're always visible on top.
    for a in map_data.get('arrows', []):
        dx, dy = a['x'] - rx, a['y'] - ry
        if math.hypot(dx, dy) <= CAM_FAR_M:
            draw_arrow_marker(surface, a['x'], a['y'], a['dir'],
                               lambda x, y: r2c(x - rx, y - ry), arrow_px)

    # Robot body
    centre = (CAM_WIN_SIZE // 2, CAM_WIN_SIZE // 2)
    pygame.draw.circle(surface, C_ROB_FILL, centre, int(ROBOT_RADIUS_M * CAM_SCALE))
    pygame.draw.circle(surface, C_ROB_RING, centre, int(ROBOT_RADIUS_M * CAM_SCALE), 1)

    # Nose dot — tracks motion_angle (actual travel direction)
    nose_r = ROBOT_RADIUS_M + 0.025
    pygame.draw.circle(surface, C_NOSE,
                        r2c(nose_r * math.cos(motion_angle),
                            nose_r * math.sin(motion_angle)), 4)

    # Camera face dots
    for i in range(NUM_CAMS):
        a = math.radians(i * 60)
        pygame.draw.circle(surface, C_CAM_DOT,
                            r2c(CAM_ARRAY_R_M * math.cos(a),
                                CAM_ARRAY_R_M * math.sin(a)), 4)

    # ── Driver decision overlays (AUTO mode only) ─────────────────────────────
    if debug_frame is not None:
        _, info = debug_frame
        cx = CAM_WIN_SIZE // 2
        cy = CAM_WIN_SIZE // 2
        # scale: info vectors are in IPM image pixels; CAM_WIN_SIZE == image size
        # so scale is 1.0 — vectors map directly to screen pixels from centre.

        def _vec(dx, dy, colour, width=2, label=None):
            ex = int(cx + dx)
            ey = int(cy + dy)
            if ex == cx and ey == cy:
                return
            pygame.draw.line(surface, colour, (cx, cy), (ex, ey), width)
            angle = math.atan2(ey - cy, ex - cx)
            for da in (+0.4, -0.4):
                ax = int(ex - 11 * math.cos(angle + da))
                ay = int(ey - 11 * math.sin(angle + da))
                pygame.draw.line(surface, colour, (ex, ey), (ax, ay), width)
            if label:
                lbl_surf = pygame.font.SysFont("monospace", 11).render(label, True, colour)
                surface.blit(lbl_surf, (ex + 4, ey - 8))

        # Dead-zone circle
        dz = int(info.get('dead_zone', 0))
        if dz > 0:
            pygame.draw.circle(surface, (70, 70, 90), (cx, cy), dz, 1)

        # Yellow wall vector
        if info.get('have_yellow'):
            _vec(info['ydx'], info['ydy'], (230, 200, 20), width=3, label='Y')

        # Blue wall vector
        if info.get('have_blue'):
            _vec(info['bdx'], info['bdy'], (80, 140, 255), width=3, label='B')

        # Midpoint / correction vector
        if info.get('have_both'):
            _vec((info['ydx'] + info['bdx']) / 2.0,
                 (info['ydy'] + info['bdy']) / 2.0,
                 (0, 220, 140), width=2, label='mid')

        # Commanded velocity arrow (scaled to ~80px)
        vx_c = info.get('cmd_vx', 0.0)
        vy_c = info.get('cmd_vy', 0.0)
        mag = math.hypot(vx_c, vy_c)
        if mag > 1e-4:
            _vec(vx_c / mag * 80, vy_c / mag * 80, (255, 90, 90), width=3, label='cmd')

        # ── Info text strip — semi-transparent bar at the bottom of the IPM ───
        _mfont  = pygame.font.SysFont("monospace", 11)
        state   = info.get('state', '?')
        hdg     = math.degrees(info.get('heading', 0.0)) % 360
        lines = [
            (f"ST: {state:<16} hdg:{hdg:.0f}°",                               (180, 180, 220)),
            (f"Y:{info.get('y_count',0):<5} B:{info.get('b_count',0):<5} "
             f"cmd:({info.get('cmd_vx',0):.2f},{info.get('cmd_vy',0):.2f})",   (180, 180, 220)),
            (f"x={info.get('rx',0):.2f} y={info.get('ry',0):.2f}",            (150, 150, 190)),
        ]
        line_h   = 13
        bar_h    = len(lines) * line_h + 4
        bar_surf = pygame.Surface((CAM_WIN_SIZE, bar_h), pygame.SRCALPHA)
        bar_surf.fill((0, 0, 0, 170))
        for i, (text, colour) in enumerate(lines):
            bar_surf.blit(_mfont.render(text, True, colour), (4, 2 + i * line_h))
        # Place bar near the bottom edge, clear of the panel label below
        surface.blit(bar_surf, (0, CAM_WIN_SIZE - bar_h - 8))

    return pygame.surfarray.array3d(surface)   # (W,H,3) for autonomous code


# ─────────────────────────────────────────────────────────────────────────────
# TOP-DOWN ARENA RENDERING
# ─────────────────────────────────────────────────────────────────────────────

def render_topdown(surface, rx, ry, heading, map_data, font):
    m2px    = map_data['m2px']
    tape_px = map_data['tape_px']
    arrow_px = map_data.get('arrow_px', tape_px)

    # Pan so the robot is centred in the view
    pan_x = int(rx * m2px - TOP_PX / 2)
    pan_y = int(ry * m2px - TOP_PY / 2)

    S = lambda x, y: w2s(x, y, m2px, pan_x, pan_y)

    surface.fill(C_BG)

    # Grid (0.5 m spacing) — draw only visible strips
    step = max(1, int(0.5 * m2px))
    # offset grids to align with world origin
    ox = (-pan_x) % step
    oy = (-pan_y) % step
    for ix in range(ox, TOP_PX + step, step):
        pygame.draw.line(surface, C_GRID, (ix, 0), (ix, TOP_PY))
    for iy in range(oy, TOP_PY + step, step):
        pygame.draw.line(surface, C_GRID, (0, iy), (TOP_PX, iy))

    # Tape — yellow outer, blue inner (green drawn AFTER cones so it's never dimmed)
    for seg in map_data['outer']:
        pygame.draw.line(surface, C_YELLOW, S(*seg[0]), S(*seg[1]), tape_px)
    for seg in map_data['inner']:
        pygame.draw.line(surface, C_BLUE,   S(*seg[0]), S(*seg[1]), tape_px)

    # Camera footprint cones (subtle alpha overlay)
    half_fov = math.radians(CAM_HFOV_DEG) / 2.0
    full_fov = half_fov * 2
    alpha_surf = pygame.Surface((TOP_PX, TOP_PY), pygame.SRCALPHA)
    for i in range(NUM_CAMS):
        cam_dir = math.radians(i * 60)
        fx = rx + CAM_ARRAY_R_M * math.cos(cam_dir)
        fy = ry + CAM_ARRAY_R_M * math.sin(cam_dir)
        pts = []
        for k in range(15):
            a = cam_dir - half_fov + full_fov * k / 14
            pts.append(S(fx + CAM_FAR_M  * math.cos(a), fy + CAM_FAR_M  * math.sin(a)))
        for k in range(14, -1, -1):
            a = cam_dir - half_fov + full_fov * k / 14
            pts.append(S(fx + CAM_NEAR_M * math.cos(a), fy + CAM_NEAR_M * math.sin(a)))
        if len(pts) >= 3:
            pygame.draw.polygon(alpha_surf, (80, 80, 130, 28), pts)
    surface.blit(alpha_surf, (0, 0))

    # Green S/F line — drawn after cones and after all other tape so it's always on top
    for seg in map_data.get('green', []):
        pygame.draw.line(surface, C_GREEN, S(*seg[0]), S(*seg[1]), tape_px)

    # Direction-arrow markers — drawn after everything else so they're always visible
    for a in map_data.get('arrows', []):
        draw_arrow_marker(surface, a['x'], a['y'], a['dir'], S, arrow_px)

    # Robot body
    rpx, rpy = S(rx, ry)
    r_px = max(4, int(ROBOT_RADIUS_M * m2px))
    pygame.draw.circle(surface, C_ROB_FILL, (rpx, rpy), r_px)
    pygame.draw.circle(surface, C_ROB_RING, (rpx, rpy), r_px, 2)

    # Nose — points in heading direction (Shift+WASD to rotate)
    nose_dx = int(math.cos(heading) * (r_px + 7))
    nose_dy = int(math.sin(heading) * (r_px + 7))
    nose_tip = (rpx + nose_dx, rpy + nose_dy)
    pygame.draw.line(surface, C_NOSE, (rpx, rpy), nose_tip, 3)
    pygame.draw.circle(surface, C_NOSE, nose_tip, 4)

    # Camera face dots
    for i in range(NUM_CAMS):
        a  = math.radians(i * 60)
        cx = rx + CAM_ARRAY_R_M * math.cos(a)
        cy = ry + CAM_ARRAY_R_M * math.sin(a)
        pygame.draw.circle(surface, C_CAM_DOT, S(cx, cy), 3)

    # HUD
    heading_name = {0.0:"East", math.pi/2:"South", math.pi:"West",
                    3*math.pi/2:"North"}.get(round(heading,4), f"{math.degrees(heading):.0f}°")
    hud = [
        f"Track: {map_data['name']}",
        f"X: {rx:.3f} m    Y: {ry:.3f} m    Heading: {heading_name}",
        "WASD = move    Shift+WASD = turn    E = reset    Q = auto/teleop    ESC = quit",
    ]
    for i, line in enumerate(hud):
        surface.blit(font.render(line, True, (70, 70, 80)), (8, 8 + i * 16))

    # Scale bar (1 m)
    bar_px = int(m2px)
    bx, by = 10, TOP_PY - 22
    pygame.draw.line(surface, (80, 80, 80), (bx, by), (bx + bar_px, by), 2)
    for ex in (bx, bx + bar_px):
        pygame.draw.line(surface, (80, 80, 80), (ex, by - 4), (ex, by + 4), 2)
    surface.blit(font.render("1 m", True, (80, 80, 80)),
                  (bx + bar_px // 2 - 12, by - 16))


# ─────────────────────────────────────────────────────────────────────────────
# MOTION COMMAND  ← swap out this function body for autonomous mode
# ─────────────────────────────────────────────────────────────────────────────

# Module-level cache so driver state is never reset mid-run
_driver_mod_cache = None

def get_motion_command(robot_state, camera_view_data):
    global _driver_mod_cache
    if _driver_mod_cache is None:
        try:
            import Holonomic_Drive.new_kiwi_sim.driverV2 as _m
            _driver_mod_cache = _m
        except Exception:
            import importlib.util, os
            driver_path = os.path.join(os.path.dirname(__file__), "driverV2.py")
            spec = importlib.util.spec_from_file_location("driverV2", driver_path)
            _m = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(_m)
            _driver_mod_cache = _m

    return _driver_mod_cache.get_motion_command(robot_state, camera_view_data)
# ─────────────────────────────────────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────

def main():
    # ── Pick track file (tkinter dialog, fully closed before pygame starts) ──
    if len(sys.argv) > 1 and os.path.isfile(sys.argv[1]):
        track_path = sys.argv[1]   # CLI shortcut: python rc_car_sim.py track.py
    else:
        track_path = pick_track_file()

    if not track_path:
        print("No track selected — exiting.")
        sys.exit(0)

    try:
        map_data = load_map_file(track_path)
    except Exception as exc:
        # Need one clean Tk() just for the error dialog
        _r = tk.Tk(); _r.withdraw()
        messagebox.showerror("Load error",
            f"Could not load track file:\n{track_path}\n\n{exc}")
        _r.destroy()
        sys.exit(1)

    # tkinter is fully gone — safe to start pygame now
    pygame.init()
    TOTAL_W = TOP_PX + 8 + CAM_WIN_SIZE
    TOTAL_H = max(TOP_PY, CAM_WIN_SIZE)
    screen  = pygame.display.set_mode((TOTAL_W, TOTAL_H))
    pygame.display.set_caption("RC Car Simulator")

    top_surf = pygame.Surface((TOP_PX, TOP_PY))
    cam_surf = pygame.Surface((CAM_WIN_SIZE, CAM_WIN_SIZE))
    font       = pygame.font.SysFont("monospace", 13)
    title_font = pygame.font.SysFont("monospace", 14, bold=True)
    clock      = pygame.time.Clock()

    rx, ry = map_data['spawn']
    # heading: angle in radians, Y-down world. 0=East, π/2=South, π=West, 3π/2=North
    heading = 0.0
    autonomous = False   # Q toggles between autonomous and teleop
    motion_angle = 0.0   # angle of the actual velocity command (vx, vy)

    # ── Lap counter ──────────────────────────────────────────────────────────
    lap_count   = 0
    yellow_hits = 0
    blue_hits   = 0
    # For each segment store the last side the robot was on (+1 / -1)
    _green_sides  = {}   # seg_index → last signed side
    _yellow_sides = {}
    _blue_sides   = {}

    while True:
        dt = clock.tick(FPS) / 1000.0

        # Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit(); sys.exit()
                if event.key == pygame.K_e:
                    rx, ry = map_data['spawn']
                    heading = 0.0
                    lap_count   = 0
                    yellow_hits = 0
                    blue_hits   = 0
                    _green_sides  = {}
                    _yellow_sides = {}
                    _blue_sides   = {}
                if event.key == pygame.K_q:
                    autonomous = not autonomous
                # Shift+WASD rotates heading (N/S/E/W only, Y-down world)
                if pygame.key.get_mods() & pygame.KMOD_SHIFT:
                    if event.key == pygame.K_d: heading = 0.0            # East
                    if event.key == pygame.K_s: heading = math.pi / 2    # South
                    if event.key == pygame.K_a: heading = math.pi        # West
                    if event.key == pygame.K_w: heading = 3 * math.pi / 2  # North

        # Camera IPM base pass — must happen before motion command (driver reads it)
        cam_data = render_camera_view(cam_surf, rx, ry, map_data, motion_angle)

        # Direction-arrow markers currently within sensing range — passed to
        # the driver alongside the rendered image so it doesn't have to infer
        # turn-warning info from pixels alone.
        visible_arrows = get_visible_arrows(rx, ry, map_data)

        # Motion
        debug_frame = None
        if autonomous:
            robot_state = {'x': rx, 'y': ry, 'heading': heading, 'arrows': visible_arrows}
            vx, vy, debug_frame = get_motion_command(robot_state, cam_data)
        else:
            keys = pygame.key.get_pressed()
            vx, vy = 0.0, 0.0
            if not (keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]):
                if keys[pygame.K_d]: vx += ROBOT_SPEED
                if keys[pygame.K_a]: vx -= ROBOT_SPEED
                if keys[pygame.K_w]: vy -= ROBOT_SPEED
                if keys[pygame.K_s]: vy += ROBOT_SPEED
            mag = math.hypot(vx, vy)
            if mag > ROBOT_SPEED:
                vx *= ROBOT_SPEED / mag
                vy *= ROBOT_SPEED / mag

        # Track motion angle for heading indicator
        if math.hypot(vx, vy) > 1e-4:
            motion_angle = math.atan2(vy, vx)

        # Integrate position
        rx = max(0.0, rx + vx * dt)
        ry = max(0.0, ry + vy * dt)

        # ── Line crossing detection ───────────────────────────────────────────
        # Fires when the robot's physical edge first contacts a line segment —
        # i.e. when the perpendicular distance from the robot centre to the
        # segment drops below ROBOT_RADIUS_M.
        # sides_dict tracks which side of each segment the robot is on so that:
        #   - It only counts once per crossing (not every frame while touching)
        #   - It won't re-count until the robot has fully cleared to the other side
        def _check_crossings(segs, sides_dict):
            count = 0
            for idx, seg in enumerate(segs):
                (x1, y1), (x2, y2) = seg
                seg_len2 = (x2-x1)**2 + (y2-y1)**2
                if seg_len2 < 1e-12:
                    continue
                seg_len = math.sqrt(seg_len2)

                # Closest point on the segment to the robot centre
                t = ((rx-x1)*(x2-x1) + (ry-y1)*(y2-y1)) / seg_len2
                if not (0.0 <= t <= 1.0):
                    continue  # robot is off the end of this segment

                # Perpendicular distance from robot centre to the segment line
                perp = ((x2-x1)*(ry-y1) - (y2-y1)*(rx-x1)) / seg_len
                perp_dist = abs(perp)

                # Which side of the line is the centre on (+1 or -1)
                curr_side = 1 if perp >= 0 else -1
                last_side = sides_dict.get(idx)

                if perp_dist <= ROBOT_RADIUS_M:
                    # Robot edge is touching/crossing the line
                    if last_side is not None and last_side != curr_side:
                        # Side has changed while touching — count one crossing
                        count += 1
                    sides_dict[idx] = curr_side
                else:
                    # Robot is clear of the line — update side freely so we're
                    # ready to detect the next approach from either direction
                    sides_dict[idx] = curr_side
            return count

        lap_count   += _check_crossings(map_data.get('green', []), _green_sides)
        yellow_hits += _check_crossings(map_data['outer'],          _yellow_sides)
        blue_hits   += _check_crossings(map_data['inner'],          _blue_sides)

        # Top-down render
        render_topdown(top_surf, rx, ry, heading, map_data, font)

        # IPM overlay pass — draw driver decision vectors onto cam_surf now that
        # debug_frame is available (couldn't do it during the base pass above)
        if debug_frame is not None:
            render_camera_view(cam_surf, rx, ry, map_data, motion_angle, debug_frame)

        # ── Composite: fixed layout, no swapping ──────────────────────────────
        screen.fill((10, 10, 14))
        cam_x = TOP_PX + 8
        cam_y = (TOTAL_H - CAM_WIN_SIZE) // 2
        screen.blit(top_surf, (0, 0))
        screen.blit(cam_surf, (cam_x, cam_y))

        # Panel labels
        screen.blit(title_font.render("TOP-DOWN VIEW",   True, (80, 80, 90)), (6, TOP_PY - 14))
        screen.blit(title_font.render("CAMERA IPM 360°", True, (80, 80, 90)), (cam_x + 6, TOTAL_H - 18))

        # Mode indicator — top-left of cam panel
        mode_text  = "MODE: AUTO" if autonomous else "MODE: TELEOP"
        mode_color = (80, 220, 80) if autonomous else (220, 120, 80)
        screen.blit(title_font.render(mode_text, True, mode_color), (cam_x + 6, 6))

        # Counters — below mode on cam panel (clear of the top-down HUD)
        screen.blit(title_font.render(f"LAPS:  {lap_count}",      True, (80, 220, 80)),  (cam_x + 6, 24))
        screen.blit(font.render(      f"YLW hits: {yellow_hits}", True, (230, 200, 20)), (cam_x + 6, 42))
        screen.blit(font.render(      f"BLU hits: {blue_hits}",   True, (80, 140, 255)), (cam_x + 6, 58))

        # FPS + angle — stacked in the top-right corner of the window
        fps_surf = font.render(f"{clock.get_fps():.0f} fps", True, (65, 105, 65))
        ang_surf = font.render(f"{math.degrees(motion_angle) % 360:.0f}°", True, (255, 100, 100))
        screen.blit(fps_surf, (TOTAL_W - fps_surf.get_width() - 4, 4))
        screen.blit(ang_surf, (TOTAL_W - ang_surf.get_width() - 4, 4 + fps_surf.get_height() + 2))

        pygame.display.flip()


if __name__ == "__main__":
    main()