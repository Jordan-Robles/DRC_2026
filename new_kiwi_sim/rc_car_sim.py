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
CAM_HFOV_DEG   = 63.0         # per-camera horizontal FOV
CAM_NEAR_M     = 0.242        # near sensing distance (m)
CAM_FAR_M      = 0.600        # far sensing distance / IPM cap (m)
NUM_CAMS       = 6

TAPE_W_M       = 0.036        # tape width (m)
TRACK_W_M      = 1.07         # full track corridor width (m)

# Colours
C_YELLOW   = (230, 200,  20)
C_BLUE     = ( 40, 110, 235)
C_BG       = ( 28,  28,  33)
C_FLOOR    = ( 52,  52,  57)
C_GRID     = ( 40,  40,  46)
C_ROB_FILL = ( 55,  55,  65)
C_ROB_RING = (210, 210, 215)
C_NOSE     = (255,  75,  75)
C_WEDGE    = ( 55,  55,  88)
C_WEDGE_BD = ( 90,  90, 140)
C_CAM_DOT  = (160, 160, 255)
C_GREEN    = (  0, 224,  96)


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

    # Scale so the track fills the view width; clamp to a readable min
    m2px    = max(20.0, TOP_PX / aw)
    tape_px = max(2, int(TAPE_W_M * m2px))
    return dict(outer=outer, inner=inner, green=green,
                spawn=(sx, sy),
                arena_w=aw, arena_h=ah, m2px=m2px,
                tape_px=tape_px, name=os.path.basename(path))


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
# CAMERA IPM RENDERING
# ─────────────────────────────────────────────────────────────────────────────

def render_camera_view(surface, rx, ry, map_data):
    """
    Render the stitched 360° IPM floor map.
    Returns a (W, H, 3) numpy array of the rendered surface.
    """
    surface.fill((14, 14, 19))
    half_fov = math.radians(CAM_HFOV_DEG) / 2.0
    full_fov = half_fov * 2
    tape_px  = max(2, int(TAPE_W_M * CAM_SCALE))

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

    # Robot body
    centre = (CAM_WIN_SIZE // 2, CAM_WIN_SIZE // 2)
    pygame.draw.circle(surface, C_ROB_FILL, centre, int(ROBOT_RADIUS_M * CAM_SCALE))
    pygame.draw.circle(surface, C_ROB_RING, centre, int(ROBOT_RADIUS_M * CAM_SCALE), 1)

    # Nose dot (+X world direction → right on IPM screen)
    pygame.draw.circle(surface, C_NOSE,
                        r2c(ROBOT_RADIUS_M + 0.025, 0), 4)

    # Camera face dots
    for i in range(NUM_CAMS):
        a = math.radians(i * 60)
        pygame.draw.circle(surface, C_CAM_DOT,
                            r2c(CAM_ARRAY_R_M * math.cos(a),
                                CAM_ARRAY_R_M * math.sin(a)), 4)

    return pygame.surfarray.array3d(surface)   # (W,H,3) for autonomous code


# ─────────────────────────────────────────────────────────────────────────────
# TOP-DOWN ARENA RENDERING
# ─────────────────────────────────────────────────────────────────────────────

def render_topdown(surface, rx, ry, heading, map_data, font):
    m2px    = map_data['m2px']
    tape_px = map_data['tape_px']

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
        surface.blit(font.render(line, True, (155, 155, 165)), (8, 8 + i * 16))

    # Scale bar (1 m)
    bar_px = int(m2px)
    bx, by = 10, TOP_PY - 22
    pygame.draw.line(surface, (170, 170, 170), (bx, by), (bx + bar_px, by), 2)
    for ex in (bx, bx + bar_px):
        pygame.draw.line(surface, (170, 170, 170), (ex, by - 4), (ex, by + 4), 2)
    surface.blit(font.render("1 m", True, (170, 170, 170)),
                  (bx + bar_px // 2 - 12, by - 16))


# ─────────────────────────────────────────────────────────────────────────────
# MOTION COMMAND  ← swap out this function body for autonomous mode
# ─────────────────────────────────────────────────────────────────────────────

def get_motion_command(robot_state, camera_view_data):
    """
    Return (vx, vy) in world-frame m/s.

    robot_state      : dict  {'x': float, 'y': float}
    camera_view_data : np.ndarray shape (W, H, 3), dtype uint8
                       This is the live IPM composite.
                       Yellow tape pixels ≈ R>180, G>160, B<80
                       Blue tape   pixels ≈ B>180, R<80

    Default: WASD teleoperation.
    W = +Y (forward),  S = −Y,  A = −X (left),  D = +X (right).
    """
    keys = pygame.key.get_pressed()
    vx, vy = 0.0, 0.0
    if not (keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]):
        if keys[pygame.K_d]: vx += ROBOT_SPEED
        if keys[pygame.K_a]: vx -= ROBOT_SPEED
        if keys[pygame.K_w]: vy -= ROBOT_SPEED   # Y-down: W = up screen = smaller Y
        if keys[pygame.K_s]: vy += ROBOT_SPEED   # Y-down: S = down screen = larger Y
    mag = math.hypot(vx, vy)
    if mag > ROBOT_SPEED:
        vx *= ROBOT_SPEED / mag
        vy *= ROBOT_SPEED / mag
    return vx, vy


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
    TOTAL_W = TOP_PX + CAM_WIN_SIZE + 8
    TOTAL_H = max(TOP_PY, CAM_WIN_SIZE)
    screen  = pygame.display.set_mode((TOTAL_W, TOTAL_H))
    pygame.display.set_caption("RC Car Simulator")

    top_surf   = pygame.Surface((TOP_PX, TOP_PY))
    cam_surf   = pygame.Surface((CAM_WIN_SIZE, CAM_WIN_SIZE))
    font       = pygame.font.SysFont("monospace", 13)
    title_font = pygame.font.SysFont("monospace", 14, bold=True)
    clock      = pygame.time.Clock()

    rx, ry = map_data['spawn']
    # heading: angle in radians, Y-down world. 0=East, π/2=South, π=West, 3π/2=North
    heading = 0.0
    autonomous = False   # Q toggles between autonomous and teleop

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
                if event.key == pygame.K_q:
                    autonomous = not autonomous
                # Shift+WASD rotates heading (N/S/E/W only, Y-down world)
                if pygame.key.get_mods() & pygame.KMOD_SHIFT:
                    if event.key == pygame.K_d: heading = 0.0            # East
                    if event.key == pygame.K_s: heading = math.pi / 2    # South
                    if event.key == pygame.K_a: heading = math.pi        # West
                    if event.key == pygame.K_w: heading = 3 * math.pi / 2  # North

        # Camera IPM (must happen before motion command)
        cam_data = render_camera_view(cam_surf, rx, ry, map_data)

        # Motion
        if autonomous:
            vx, vy = get_motion_command({'x': rx, 'y': ry, 'heading': heading}, cam_data)
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

        # Integrate position
        rx = max(0.0, rx + vx * dt)
        ry = max(0.0, ry + vy * dt)

        # Top-down render
        render_topdown(top_surf, rx, ry, heading, map_data, font)

        # Composite
        screen.fill((10, 10, 14))
        screen.blit(top_surf, (0, 0))
        screen.blit(cam_surf, (TOP_PX + 8, (TOTAL_H - CAM_WIN_SIZE) // 2))

        # Panel labels
        screen.blit(title_font.render("TOP-DOWN VIEW",   True, (190, 190, 200)), (6, TOP_PY - 14))
        screen.blit(title_font.render("CAMERA IPM 360°", True, (190, 190, 200)),
                    (TOP_PX + 12, TOTAL_H - 18))

        # Autonomous mode indicator
        mode_text  = "MODE: AUTO" if autonomous else "MODE: TELEOP"
        mode_color = (80, 220, 80) if autonomous else (220, 120, 80)
        screen.blit(title_font.render(mode_text, True, mode_color), (TOP_PX + 12, 6))

        # FPS counter
        screen.blit(font.render(f"{clock.get_fps():.0f} fps", True, (65, 105, 65)),
                    (TOTAL_W - 58, 4))

        pygame.display.flip()


if __name__ == "__main__":
    main()
