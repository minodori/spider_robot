#!/usr/bin/env python3
"""
Spider Robot Walk Simulation - PyBullet (Physical)
Reproduces WALK() Crawl V1.0 gait from main.cpp
Physical simulation: body unconstrained, robot walks forward.

Legs: A(front-left), B(front-right), C(rear-left), D(rear-right)
Gait order: A -> D -> B -> C (static creep)

Camera controls (PyBullet GUI):
  Left drag    : rotate
  Right drag   : pan
  Scroll wheel : zoom
  Ctrl + drag  : pan

Keyboard controls:
  Space        : pause / resume
  R            : restart (return to stall position)
  Q / Esc      : quit
"""

import pybullet as p
import pybullet_data
import math
import time
import os
import tempfile
from spider_urdf import URDF_STR

# ─── Parameters from main.cpp ────────────────────────────────────────────────
PHY_CONTACT = [100, 80, 30, 150]  # contact angle per leg [A, B, C, D]
ARM_DIR = [1, -1, 1, -1]  # stance direction
STRIDE = 50  # stance range (deg)
PHY_STL = [90, 90, 90, 90]  # neutral arm angle (deg)
FDW = 30  # foot down (standing)
FUP = 60  # foot up (lifted)

# ─── Crawl gait state machine (Crawl V1.0) ───────────────────────────────────
# All 4 legs run simultaneously: 1 swings, 3 push back.
# Order: A(0) → D(25) → B(50) → C(75)  within 100-step cycle.
CRAWL_CYCLE = 100  # steps per full gait cycle
SWING_STEPS = 25  # steps each leg is airborne (= 1/4 cycle)
# Step within cycle when each leg starts its swing [A, B, C, D]
SWING_START = [0, 50, 75, 25]


def lerp(a, b, t):
    return a + (b - a) * t


def crawl_arm_target(leg, step):
    """Arm angle (physical deg) for crawl state-machine step."""
    offset = (step - SWING_START[leg]) % CRAWL_CYCLE
    if offset < SWING_STEPS:  # swing: toeoff → contact
        return lerp(phy_toeoff(leg), PHY_CONTACT[leg], offset / SWING_STEPS)
    else:  # stance: contact → toeoff
        t = (offset - SWING_STEPS) / (CRAWL_CYCLE - SWING_STEPS)
        return lerp(PHY_CONTACT[leg], phy_toeoff(leg), t)


def crawl_foot_target(leg, step):
    """Foot angle (physical deg) for crawl state-machine step."""
    offset = (step - SWING_START[leg]) % CRAWL_CYCLE
    if offset < SWING_STEPS:
        t = offset / SWING_STEPS
        return lerp(FDW, FUP, t * 2.0) if t < 0.5 else lerp(FUP, FDW, (t - 0.5) * 2.0)
    return FDW


def phy_toeoff(leg):
    return PHY_CONTACT[leg] + ARM_DIR[leg] * STRIDE


# ─── Angle conversion ────────────────────────────────────────────────────────
def arm_sim(leg, phy_deg):
    """Physical servo angle -> PyBullet joint angle (rad)."""
    return math.radians(phy_deg - PHY_STL[leg])


def foot_sim(phy_deg):
    """Foot servo angle -> PyBullet joint angle (rad).
    Negative: foot swings outward (+X in arm frame) = lifts up.
    Physical ref: FDW(30)=down, FUP(60)=lifted, 120=horizontal."""
    return -math.radians(phy_deg - FDW)


# ─── PyBullet setup ──────────────────────────────────────────────────────────
PHYSICS_HZ = 240

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0 / PHYSICS_HZ)
p.resetDebugVisualizerCamera(
    cameraDistance=0.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.1]
)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

plane = p.loadURDF("plane.urdf")
p.changeDynamics(
    plane, -1, lateralFriction=3.0, spinningFriction=0.2, rollingFriction=0.2
)

tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
tmp.write(URDF_STR)
tmp.close()
try:
    robot = p.loadURDF(tmp.name, [0, 0, 0.15], p.getQuaternionFromEuler([0, 0, 0]))
finally:
    os.unlink(tmp.name)

# Body damping: resist tipping when legs push asymmetrically
p.changeDynamics(robot, -1, linearDamping=0.4, angularDamping=0.9)

# ─── Joint index map ─────────────────────────────────────────────────────────
jmap = {}
for i in range(p.getNumJoints(robot)):
    name = p.getJointInfo(robot, i)[1].decode()
    jmap[name] = i

ARM = [jmap["arm_a"], jmap["arm_b"], jmap["arm_c"], jmap["arm_d"]]
FOOT = [jmap["foot_a"], jmap["foot_b"], jmap["foot_c"], jmap["foot_d"]]

# Increase foot friction and add joint damping for stability
for i in range(p.getNumJoints(robot)):
    p.changeDynamics(
        robot,
        i,
        lateralFriction=2.0,
        spinningFriction=0.1,
        rollingFriction=0.1,
        jointDamping=0.5,
    )

# ─── Debug UI ────────────────────────────────────────────────────────────────
speed_id = p.addUserDebugParameter("Speed (1=slow, 10=fast)", 1, 10, 1)

# Status text (top-left)
status_id = p.addUserDebugText("Initializing...", [0, 0, 0.22], [1, 1, 1], 1.2)

# ─── State ───────────────────────────────────────────────────────────────────
cur_arm = list(PHY_STL)
cur_foot = [FDW, FDW, FDW, FDW]
paused = False

KEY_SPACE = ord(" ")
KEY_R = ord("r")
KEY_Q = ord("q")
KEY_ESC = 27


def get_step_delay():
    speed = p.readUserDebugParameter(speed_id)  # 1..10
    return (11 - speed) * 0.001  # 10ms..1ms


def apply_joints(step=None):
    """Apply crawl state-machine targets (step) or stall targets (step=None)."""
    for leg in range(4):
        if step is None:
            arm_t = arm_sim(leg, cur_arm[leg])
            foot_t = foot_sim(cur_foot[leg])
        else:
            arm_t = arm_sim(leg, crawl_arm_target(leg, step))
            foot_t = foot_sim(crawl_foot_target(leg, step))
        p.setJointMotorControl2(
            robot,
            ARM[leg],
            p.POSITION_CONTROL,
            targetPosition=arm_t,
            force=60,
            maxVelocity=3,
        )
        p.setJointMotorControl2(
            robot,
            FOOT[leg],
            p.POSITION_CONTROL,
            targetPosition=foot_t,
            force=60,
            maxVelocity=3,
        )


def update_camera():
    pos, _ = p.getBasePositionAndOrientation(robot)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[pos[0], pos[1], 0.1],
    )


def sim_step():
    """One simulation step. Returns False if quit was requested."""
    global paused

    keys = p.getKeyboardEvents()

    # Toggle pause
    if KEY_SPACE in keys and keys[KEY_SPACE] & p.KEY_WAS_TRIGGERED:
        paused = not paused
        label = "PAUSED  [Space]=resume  [R]=restart  [Q]=quit"
        if paused:
            print("[Space] Paused")
            p.addUserDebugText(
                label, [0, 0, 0.22], [1, 0.4, 0.4], 1.2, replaceItemUniqueId=status_id
            )
        else:
            print("[Space] Resumed")

    # Quit
    if (KEY_Q in keys and keys[KEY_Q] & p.KEY_WAS_TRIGGERED) or (
        KEY_ESC in keys and keys[KEY_ESC] & p.KEY_WAS_TRIGGERED
    ):
        return False

    if paused:
        p.stepSimulation()
        time.sleep(0.016)

    return True


def stall():
    """Settle robot at crawl step=0 (leg A at contact, ready to push)."""
    for leg in range(4):
        cur_arm[leg] = PHY_STL[leg]
        cur_foot[leg] = FDW
    for _ in range(300):  # longer settle for physics stability
        apply_joints(step=0)
        p.stepSimulation()
        time.sleep(1.0 / PHYSICS_HZ)
    p.resetBaseVelocity(robot, [0, 0, 0], [0, 0, 0])


LEG_NAMES = ["A(FL)", "B(FR)", "C(RL)", "D(RR)"]
# Which leg is swinging at each step (for status display)
_SWING_LEG_AT = {}
for _leg in range(4):
    for _s in range(SWING_STEPS):
        _SWING_LEG_AT[(SWING_START[_leg] + _s) % CRAWL_CYCLE] = _leg


# ─── Main ────────────────────────────────────────────────────────────────────
print("=" * 52)
print("  Spider Robot Walk Simulation")
print("=" * 52)
print("  Camera : left-drag=rotate  right-drag=pan  scroll=zoom")
print("  Keys   : [Space]=pause/resume  [R]=restart  [Q]=quit")
print()
print("  Gait parameters:")
for i, n in enumerate("ABCD"):
    print(
        f"    Leg {n}: contact={PHY_CONTACT[i]:3d} deg  "
        f"toeoff={phy_toeoff(i):3d} deg  "
        f"dir={'CW' if ARM_DIR[i]==1 else 'CCW'}"
    )
print()

stall()

crawl_step = 0
cycle = 0
prev_swing = -1

try:
    while p.isConnected():
        keys = p.getKeyboardEvents()

        # Restart
        if KEY_R in keys and keys[KEY_R] & p.KEY_WAS_TRIGGERED:
            print("[R] Restarting...")
            pos, _ = p.getBasePositionAndOrientation(robot)
            p.resetBasePositionAndOrientation(
                robot, [pos[0], pos[1], 0.15], p.getQuaternionFromEuler([0, 0, 0])
            )
            p.resetBaseVelocity(robot, [0, 0, 0], [0, 0, 0])
            stall()
            crawl_step = 0
            cycle = 0
            prev_swing = -1
            continue

        if not sim_step():
            break

        if paused:
            continue

        # Apply coordinated crawl targets for all 4 legs
        apply_joints(step=crawl_step)
        for _ in range(10):
            p.stepSimulation()
        time.sleep(get_step_delay())

        # Status: announce when a new swing starts
        sw = _SWING_LEG_AT.get(crawl_step, -1)
        if sw != prev_swing and sw != -1:
            label = (
                f"Leg {LEG_NAMES[sw]} swinging  "
                f"contact={PHY_CONTACT[sw]}  toeoff={phy_toeoff(sw)}"
            )
            print(f"  {label}")
            pos, _ = p.getBasePositionAndOrientation(robot)
            p.addUserDebugText(
                label,
                [pos[0], pos[1], 0.22],
                [0.3, 1.0, 0.4],
                1.2,
                replaceItemUniqueId=status_id,
            )
            prev_swing = sw

        crawl_step = (crawl_step + 1) % CRAWL_CYCLE
        if crawl_step == 0:
            cycle += 1
            print(f"=== Cycle {cycle} ===")

        update_camera()

except (SystemExit, KeyboardInterrupt):
    pass

if p.isConnected():
    p.disconnect()
print("Simulation ended.")
