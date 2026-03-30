#!/usr/bin/env python3
"""
Spider Robot – Trot Simulation (PyBullet)
Gait: 3/4-delay 4-beat walk from docs/trot.csv (reference video frames 681-704)
Cycle: 24 steps  |  Phase offsets: FL=15, FR=4, BL=10, BR=23

Physical simulation: body unconstrained, robot walks forward.

Camera controls:
  Left drag    : rotate
  Right drag   : pan
  Scroll wheel : zoom

Keyboard controls:
  Space        : pause / resume
  R            : restart
  Q / Esc      : quit
"""

import pybullet as p
import pybullet_data
import math
import time
import os

# ── Servo / gait parameters ────────────────────────────────────────────────
# Legs: 0=A(FL)  1=B(FR)  2=C(BL)  3=D(BR)
PHY_CONTACT = [100,  80,  30, 150]   # arm angle at ground contact (deg)
ARM_DIR     = [  1,  -1,   1,  -1]   # +1=CW, -1=CCW during stance
STRIDE      = 50                      # contact → toeoff range (deg)
PHY_STL     = [ 90,  90,  90,  90]   # neutral arm angle (deg)
FDW, FUP    = 30, 60                  # foot: down / lifted

def phy_toeoff(leg):
    return PHY_CONTACT[leg] + ARM_DIR[leg] * STRIDE

# ── Trot.csv timing (0-indexed, within 24-frame cycle) ─────────────────────
# Derived from docs/trot.csv (frames 681-704, FL starts at Place-Down)
CYCLE_LEN   = 24
SWING_START = [15,  4, 10, 23]   # step when leg starts lifting  [FL,FR,BL,BR]
SWING_LEN   = [ 9,  7,  6,  6]   # steps airborne

LEG_NAMES   = ['A(FL)', 'B(FR)', 'C(BL)', 'D(BR)']

# ── Physics timing ─────────────────────────────────────────────────────────
PHYSICS_HZ       = 240
GAIT_STEP_SEC    = 0.08          # 24 steps × 0.08 s ≈ 1.9 s per cycle (default speed)
PHYSICS_PER_STEP = int(PHYSICS_HZ * GAIT_STEP_SEC)   # physics steps per gait step

# ── Angle helpers ──────────────────────────────────────────────────────────
def arm_sim(leg, phy_deg):
    """Physical degrees → PyBullet joint angle (rad)."""
    return math.radians(phy_deg - PHY_STL[leg])

def foot_sim(phy_deg):
    """Physical degrees → PyBullet joint angle (rad). Negative = foot lifts."""
    return -math.radians(phy_deg - FDW)

def lerp(a, b, t):
    return a + (b - a) * t

# ── Gait phase → joint targets ─────────────────────────────────────────────
def arm_target(leg, step):
    """Target arm angle (physical deg) at gait step."""
    offset = (step - SWING_START[leg]) % CYCLE_LEN
    sw = SWING_LEN[leg]
    if offset < sw:                          # swing: toeoff → contact
        return lerp(phy_toeoff(leg), PHY_CONTACT[leg], offset / sw)
    else:                                    # stance: contact → toeoff
        return lerp(PHY_CONTACT[leg], phy_toeoff(leg),
                    (offset - sw) / (CYCLE_LEN - sw))

def foot_target(leg, step):
    """Target foot angle (physical deg) at gait step."""
    offset = (step - SWING_START[leg]) % CYCLE_LEN
    sw = SWING_LEN[leg]
    if offset < sw:
        t = offset / sw
        if t < 0.5:
            return lerp(FDW, FUP, t * 2.0)
        else:
            return lerp(FUP, FDW, (t - 0.5) * 2.0)
    return FDW

# ── PyBullet setup ─────────────────────────────────────────────────────────
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0 / PHYSICS_HZ)
p.resetDebugVisualizerCamera(
    cameraDistance=0.6,
    cameraYaw=45,
    cameraPitch=-25,
    cameraTargetPosition=[0, 0, 0.1]
)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

plane = p.loadURDF("plane.urdf")
p.changeDynamics(plane, -1, lateralFriction=1.5, spinningFriction=0.05, rollingFriction=0.05)

_URDF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "spider.urdf")
robot = p.loadURDF(_URDF_PATH, [0, 0, 0.15], p.getQuaternionFromEuler([0, 0, 0]))

# Increase foot friction and add joint damping for stability
for i in range(p.getNumJoints(robot)):
    p.changeDynamics(robot, i, lateralFriction=2.0, spinningFriction=0.1, rollingFriction=0.1,
                     jointDamping=0.5)

# ── Joint index map ────────────────────────────────────────────────────────
jmap = {}
for i in range(p.getNumJoints(robot)):
    name = p.getJointInfo(robot, i)[1].decode()
    jmap[name] = i

ARM  = [jmap['arm_a'],  jmap['arm_b'],  jmap['arm_c'],  jmap['arm_d']]
FOOT = [jmap['foot_a'], jmap['foot_b'], jmap['foot_c'], jmap['foot_d']]

# ── Debug UI ───────────────────────────────────────────────────────────────
speed_id  = p.addUserDebugParameter("Speed (1=slow, 10=fast)", 1, 10, 5)
status_id = p.addUserDebugText("Settling...", [0, 0, 0.25], [1, 1, 1], 1.2)

# ── State ──────────────────────────────────────────────────────────────────
paused     = False
gait_step  = 0
cycle      = 0

KEY_SPACE = ord(' ')
KEY_R     = ord('r')
KEY_Q     = ord('q')
KEY_ESC   = 27

def get_physics_per_step():
    speed = p.readUserDebugParameter(speed_id)   # 1..10
    # speed 1 → 0.12 s/step (slow),  speed 10 → 0.03 s/step (fast)
    sec = lerp(0.12, 0.03, (speed - 1) / 9.0)
    return max(1, int(PHYSICS_HZ * sec))

def apply_joints(step):
    for leg in range(4):
        p.setJointMotorControl2(
            robot, ARM[leg], p.POSITION_CONTROL,
            targetPosition=arm_sim(leg, arm_target(leg, step)),
            force=60, maxVelocity=3)
        p.setJointMotorControl2(
            robot, FOOT[leg], p.POSITION_CONTROL,
            targetPosition=foot_sim(foot_target(leg, step)),
            force=60, maxVelocity=3)

def update_camera():
    pos, _ = p.getBasePositionAndOrientation(robot)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.6,
        cameraYaw=45,
        cameraPitch=-25,
        cameraTargetPosition=[pos[0], pos[1], 0.1]
    )

def check_keys():
    global paused
    keys = p.getKeyboardEvents()
    if KEY_SPACE in keys and keys[KEY_SPACE] & p.KEY_WAS_TRIGGERED:
        paused = not paused
        print("[Space]", "Paused" if paused else "Resumed")
    if (KEY_Q in keys and keys[KEY_Q] & p.KEY_WAS_TRIGGERED) or \
       (KEY_ESC in keys and keys[KEY_ESC] & p.KEY_WAS_TRIGGERED):
        return False
    return True

# ── Settle under gravity ───────────────────────────────────────────────────
# Set stall pose (step 0 targets) and let physics settle
for _ in range(120):
    apply_joints(0)
    p.stepSimulation()
    time.sleep(1.0 / PHYSICS_HZ)

print("=" * 52)
print("  Spider Robot Trot Simulation")
print("=" * 52)
print("  Gait: 3/4-delay walk (trot.csv, 24-step cycle)")
print()
print("  Swing offsets  (step in cycle when leg lifts):")
for i, n in enumerate(LEG_NAMES):
    print(f"    {n}: lifts at step {SWING_START[i]:2d}, "
          f"airborne {SWING_LEN[i]} steps, "
          f"contact={PHY_CONTACT[i]} toeoff={phy_toeoff(i)}")
print()
print("  [Space]=pause/resume  [R]=restart  [Q]=quit")
print()

p.addUserDebugText("Running – Cycle 1", [0, 0, 0.25], [0.3, 1.0, 0.4], 1.2,
                   replaceItemUniqueId=status_id)

# ── Main loop ──────────────────────────────────────────────────────────────
try:
    while p.isConnected():
        if not check_keys():
            break

        # Restart
        keys = p.getKeyboardEvents()
        if KEY_R in keys and keys[KEY_R] & p.KEY_WAS_TRIGGERED:
            print("[R] Restarting...")
            pos, _ = p.getBasePositionAndOrientation(robot)
            p.resetBasePositionAndOrientation(
                robot, [pos[0], pos[1], 0.15], p.getQuaternionFromEuler([0, 0, 0]))
            p.resetBaseVelocity(robot, [0, 0, 0], [0, 0, 0])
            gait_step = 0
            cycle = 0
            for _ in range(120):
                apply_joints(0)
                p.stepSimulation()
                time.sleep(1.0 / PHYSICS_HZ)
            continue

        if paused:
            apply_joints(gait_step)
            p.stepSimulation()
            time.sleep(0.016)
            continue

        # Apply targets for current gait step
        apply_joints(gait_step)

        # Step physics
        n = get_physics_per_step()
        dt = 1.0 / PHYSICS_HZ
        for _ in range(n):
            p.stepSimulation()
            time.sleep(dt)

        # Advance gait
        gait_step = (gait_step + 1) % CYCLE_LEN
        if gait_step == 0:
            cycle += 1
            label = f"Cycle {cycle}"
            print(f"=== {label} ===")
            pos, _ = p.getBasePositionAndOrientation(robot)
            p.addUserDebugText(label, [0, 0, 0.25], [1, 1, 1], 1.2,
                               replaceItemUniqueId=status_id)

        update_camera()

except (SystemExit, KeyboardInterrupt):
    pass

if p.isConnected():
    p.disconnect()
print("Simulation ended.")
