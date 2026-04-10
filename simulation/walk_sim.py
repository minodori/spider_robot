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

# ─── Parameters from main.cpp ────────────────────────────────────────────────
# Angles in URDF world-frame degrees: 0=front(+x), CCW positive, right-hand rule
MOUNT_YAW = [45, -45, 135, -135]  # leg mount yaw [A=FL, B=FR, C=BL, D=BR]
PHY_CONTACT = [55, -55, 75, -75]  # world-frame contact angle
#   FL: mid= 80°, sin=0.98  FR: mid=-80°, sin=0.98
#   BL: mid=100°, sin=0.98  BR: mid=-100°, sin=0.98  → 전진 기여 균등
PHY_STL = MOUNT_YAW  # neutral (stall) = mount direction
ARM_DIR = [1, -1, 1, -1]  # +1=CCW, -1=CW during stance
STRIDE = 50  # angular stride (deg)
FDW = 30  # foot down (standing)
FUP = 60  # foot up (lifted)

# ─── Crawl gait state machine (Crawl V1.0) ───────────────────────────────────
# All 4 legs run simultaneously: 1 swings, 3 push back.
# Order: A(FL,0) → D(BR,25) → B(FR,50) → C(BL,75)  within 100-step cycle.
CRAWL_CYCLE = 100  # steps per full gait cycle
SWING_STEPS = 25  # steps each leg is airborne (= 1/4 cycle)
# Step within cycle when each leg starts its swing [A=FL, B=FR, C=BL, D=BR]
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
def arm_sim(leg, world_deg):
    """World-frame arm angle (deg) -> PyBullet joint angle (rad).
    PyBullet revolute joint=0 when arm points along mount_yaw direction."""
    return math.radians(world_deg - MOUNT_YAW[leg])


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

_URDF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "spider.urdf")
robot = p.loadURDF(_URDF_PATH, [0, 0, 0.15], p.getQuaternionFromEuler([0, 0, 0]))

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
speed_id = p.addUserDebugParameter("Speed (0.1=slow, 10=fast)", 0.1, 10, 0.1)
step_btn_id = p.addUserDebugParameter("[S] Step (drag right = +1 frame)", 0, 9999, 0)

# Status text (top-left)
status_id = p.addUserDebugText("Initializing...", [0, 0, 0.22], [1, 1, 1], 1.2)

# ─── State ───────────────────────────────────────────────────────────────────
cur_arm = list(PHY_STL)
cur_foot = [FDW, FDW, FDW, FDW]
paused = True   # start paused so user can step from the very first frame
prev_step_btn = 0

KEY_SPACE = ord(" ")
KEY_R = ord("r")
KEY_Q = ord("q")
KEY_S = ord("s")
KEY_ESC = 27


def get_step_delay():
    speed = p.readUserDebugParameter(speed_id)  # 0.1..10
    return max(0.0, (10 - speed) / 10 * 0.05)  # 50ms(slow)..0ms(fast)


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
    """One simulation step. Returns (continue, do_gait_step)."""
    global paused, prev_step_btn

    keys = p.getKeyboardEvents()

    # Toggle pause
    if KEY_SPACE in keys and keys[KEY_SPACE] & p.KEY_WAS_TRIGGERED:
        paused = not paused
        if paused:
            print("[Space] Paused  — [S]=step  [Space]=resume")
        else:
            print("[Space] Resumed")

    # Quit
    if (KEY_Q in keys and keys[KEY_Q] & p.KEY_WAS_TRIGGERED) or (
        KEY_ESC in keys and keys[KEY_ESC] & p.KEY_WAS_TRIGGERED
    ):
        return False, False

    if paused:
        # S key: advance exactly one gait frame
        s_key = KEY_S in keys and keys[KEY_S] & p.KEY_WAS_TRIGGERED
        # Slider button: each integer increment = one gait frame
        btn_val = int(p.readUserDebugParameter(step_btn_id))
        slider_step = btn_val > prev_step_btn
        if slider_step:
            prev_step_btn = btn_val

        if s_key or slider_step:
            return True, True  # advance one gait step

        p.stepSimulation()
        time.sleep(0.016)
        return True, False

    return True, True


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


LEG_NAMES = ["A(FL)", "B(FR)", "C(BL)", "D(BR)"]
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
print("  Keys   : [Space]=pause/resume  [S]=single-step  [R]=restart  [Q]=quit")
print()
print("  Gait parameters:")
for i, n in enumerate("ABCD"):
    print(
        f"    Leg {n}: contact={PHY_CONTACT[i]:3d} deg  "
        f"toeoff={phy_toeoff(i):3d} deg  "
        f"dir={'CCW' if ARM_DIR[i]==1 else 'CW'}"
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

        ok, do_step = sim_step()
        if not ok:
            break

        if not do_step:
            continue

        # Apply coordinated crawl targets for all 4 legs
        apply_joints(step=crawl_step)

        if paused:
            # Single-step: run a few physics steps so the joints settle visually
            for _ in range(30):
                p.stepSimulation()
        else:
            for _ in range(10):
                p.stepSimulation()
            time.sleep(get_step_delay())

        # Status: show current gait step and which leg is swinging
        sw = _SWING_LEG_AT.get(crawl_step, -1)
        swing_txt = f"swing={LEG_NAMES[sw]}" if sw != -1 else "stance only"
        label = (
            f"step={crawl_step:3d}  cycle={cycle}  {swing_txt}"
        )
        if paused:
            label = "[PAUSED] " + label + "  [S]=next  [Space]=run"
        if sw != prev_swing and sw != -1:
            print(f"  step={crawl_step}: Leg {LEG_NAMES[sw]} swinging  "
                  f"contact={PHY_CONTACT[sw]}  toeoff={phy_toeoff(sw)}")
            prev_swing = sw
        pos, _ = p.getBasePositionAndOrientation(robot)
        p.addUserDebugText(
            label,
            [pos[0], pos[1], 0.22],
            [1.0, 0.8, 0.2] if paused else [0.3, 1.0, 0.4],
            1.2,
            replaceItemUniqueId=status_id,
        )

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
