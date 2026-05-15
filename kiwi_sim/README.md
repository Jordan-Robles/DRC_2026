# Kiwi Drive Webots Sim

Locked-rotation holonomic kiwi drive robot on a blank 8×8 m arena.
Built for macOS (Apple Silicon M1) + Webots R2023b.

---

## 1. Install Webots

Download the **Apple Silicon (arm64)** build from the official releases page:

```
https://github.com/cyberbotics/webots/releases/tag/R2023b
```

Download: `webots-R2023b-arm64.dmg`  
Drag `Webots.app` into `/Applications`.

> **Do not use Homebrew** — it installs the x86 build which runs under Rosetta
> and is significantly slower.

---

## 2. Open the World

1. Launch **Webots**
2. `File → Open World…`
3. Navigate to `kiwi_sim/worlds/kiwi_arena.wbt`
4. The first load will download two EXTERNPROTO assets (background textures)
   — this takes a few seconds with internet access.

---

## 3. Verify the Controller is Linked

The world file sets `controller "kiwi_controller"`. Webots resolves this by
looking for a folder named `kiwi_controller` inside a `controllers/` directory
**at the same level as** the `worlds/` folder.

Your layout should be:

```
kiwi_sim/
├── worlds/
│   └── kiwi_arena.wbt
└── controllers/
    └── kiwi_controller/
        └── kiwi_controller.py
```

If Webots says "controller not found", go to:  
`Tools → Preferences → Python command`  
and make sure it points to a Python 3.x binary (Webots ships its own — it
auto-detects on first launch).

---

## 4. Run & Drive

Press **▶ Play** (or Space).

| Key | Action |
|-----|--------|
| W   | Forward |
| S   | Backward |
| A   | Strafe left |
| D   | Strafe right |
| W+D | Diagonal forward-right |
| (any two cardinal keys) | Full diagonal |

Click inside the **3D viewport first** to ensure it captures keyboard input.

Telemetry prints to the **Webots console** (bottom panel) every ~1 second of
sim time.

---

## 5. Switching to Autonomous Mode

Open `controllers/kiwi_controller/kiwi_controller.py` and change line 8:

```python
# Before
MODE = "teleop"

# After
MODE = "auto"
```

Then fill in `autonomous_source()` with your pipeline output.  
The function receives a `sensors` dict:

```python
sensors = {
    "gps":  (x, y, z),   # metres, world frame
    "gyro": (wx, wy, wz), # rad/s
    "time": float,        # simulation seconds
}
```

It must return a `MotionCommand(vx, vy, omega)` where `vx`/`vy` are in **m/s
in the world frame** (+X = right, +Y = forward).  
`omega` is accepted in the dataclass but is **forced to 0** by the IK solver
— the robot will never yaw, matching your locked-rotation hardware setup.

---

## 6. Kiwi IK Reference

The three wheels are placed at 120° intervals.  
Wheel 0 (front) has its drive direction along **+Y** (forward).

For wheel *i* with drive-direction angle θ_i:

```
wheel_speed_i = (vx·cos(θ_i) + vy·sin(θ_i)) / r_wheel
```

| Wheel | Position | Drive angle θ |
|-------|----------|---------------|
| 0 | Front | 90° |
| 1 | Rear-right | 210° |
| 2 | Rear-left | 330° |

Wheel radius `r = 0.025 m`, base radius `R = 0.09 m`.

---

## 7. Next Steps (suggested order)

1. **Add camera nodes** — 6× `Camera` devices in the `.wbt` at the correct
   hexagonal positions and tilt angles (30° down), matching your physical rig.
2. **Add a test track** — paint tape lines using `TexturedBox` objects or a
   custom floor texture.
3. **Wire IPM pipeline** — `autonomous_source()` reads camera images from
   Webots, runs your OpenCV homography warp, and returns a `MotionCommand`.
4. **Add a GPS ground-truth overlay** — the `gps` sensor is already enabled;
   use it to log cross-track error vs your IPM estimate.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Robot spins instead of translating | Webots physics timestep mismatch — confirm `basicTimeStep 16` in .wbt |
| "No controller" warning | Ensure folder name `kiwi_controller` matches exactly (case-sensitive) |
| Keyboard not responding | Click the 3D viewport to give it focus before pressing keys |
| Slow simulation on M1 | `Preferences → OpenGL → Disable shadows` — big speedup |
| EXTERNPROTO download fails | Manually copy the two texture protos offline; ask for instructions |
