#!/usr/bin/env python3
"""
Spider Robot Walk Simulation - PyBullet
Logical angle coordinate system:
  0=forward, 90=horizontal, 180=backward, CCW=positive
Legs: A(FL), B(FR), C(BL), D(BR)  Gait: A->D->B->C
Keys: Space=pause/run  S=single step  R=restart  Q=quit
"""

import pybullet as p
import pybullet_data
import math
import time
import os

# ─── Logical angle parameters ─────────────────────────────────────────────────
# Arm ranges: A:-45~135  B:-135~45  C:45~225  D:-225~-45
LOG_STL = [45, -45, 135, -135]  # neutral (diagonal)
LOG_CONTACT = [75, -75, 75, -75]  # contact  (horiz -15°, forward)
LOG_TOEOFF = [125, -125, 125, -125]  # toeoff   (horiz +35°, backward)
STRIDE = 50  # total stride (deg)
ARM_LIMIT = 15  # safety margin each end (deg)

# Foot logical angles (all legs identical)
# phy = 210 - log
LOG_FDW = 170  # standing,   phy=40
# LOG_FDW = 120  # standing,   phy=40
LOG_STL_F = 135  # stall/init, phy=75
# LOG_STL_F = 120  # stall/init, phy=75
# LOG_FUP = 90  # body low,   phy=90
LOG_FUP = 120  # body low,   phy=90
LOG_FOOT_MIN = 30  # max down,   phy=180
LOG_FOOT_MAX = 210  # max up,     phy=0 (limit으로 차단)
FOOT_LIMIT = 10  # safety margin each end (deg)

# ─── Crawl gait ───────────────────────────────────────────────────────────────
CRAWL_CYCLE = 100
SWING_STEPS = 25
SWING_START = [0, 50, 75, 25]  # [A, B, C, D]


# ─── Angle conversion ─────────────────────────────────────────────────────────
def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def arm_log_range(leg):
    ranges = [
        (-45 + ARM_LIMIT, 135 - ARM_LIMIT),
        (-135 + ARM_LIMIT, 45 - ARM_LIMIT),
        (45 + ARM_LIMIT, 225 - ARM_LIMIT),
        (-225 + ARM_LIMIT, -45 - ARM_LIMIT),
    ]
    return ranges[leg]


def to_servo_arm(leg, logical):
    lo, hi = arm_log_range(leg)
    logical = clamp(logical, lo, hi)
    return [logical + 45, logical + 135, logical - 45, logical + 225][leg]


def to_servo_foot(logical):
    lo = LOG_FOOT_MIN + FOOT_LIMIT
    hi = LOG_FOOT_MAX - FOOT_LIMIT
    return 210 - clamp(logical, lo, hi)


def arm_sim(leg, logical):
    """Logical deg -> PyBullet joint rad.
    URDF joint=0 when arm points along mount_yaw.
    mount_yaw (logical): A=45 B=-45 C=135 D=-135"""
    MOUNT = [45, -45, 135, -135]
    # return math.radians(-(logical - MOUNT[leg]))
    return math.radians(logical - MOUNT[leg])
    # sign = [1, -1, 1, -1]  # B/D URDF joint 반전
    # return math.radians(sign[leg] * (logical - MOUNT[leg]))


def foot_sim(logical):
    """Logical deg -> PyBullet joint rad.
    phy=90(log=120)=horizontal=joint 0.
    FDW(log=170,phy=40): (40-90)=-0.5rad → foot down (outward)."""
    phy = to_servo_foot(logical)
    return math.radians(phy - 90)


# ─── Gait functions ───────────────────────────────────────────────────────────
def lerp(a, b, t):
    return a + (b - a) * t


def crawl_arm_target(leg, step):
    offset = (step - SWING_START[leg]) % CRAWL_CYCLE
    if offset < SWING_STEPS:
        return lerp(LOG_TOEOFF[leg], LOG_CONTACT[leg], offset / SWING_STEPS)
    else:
        t = (offset - SWING_STEPS) / (CRAWL_CYCLE - SWING_STEPS)
        return lerp(LOG_CONTACT[leg], LOG_TOEOFF[leg], t)


def crawl_foot_target(leg, step):
    offset = (step - SWING_START[leg]) % CRAWL_CYCLE
    if offset < SWING_STEPS:
        t = offset / SWING_STEPS
        return (
            lerp(LOG_FDW, LOG_FUP, t * 2.0)
            if t < 0.5
            else lerp(LOG_FUP, LOG_FDW, (t - 0.5) * 2.0)
        )
    return LOG_FDW


# ─── PyBullet setup ───────────────────────────────────────────────────────────
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

_URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "spider.urdf")
# robot = p.loadURDF(_URDF, [0, 0, 0.15], p.getQuaternionFromEuler([0, 0, math.pi / 2]))  # 90도 회전
robot = p.loadURDF(_URDF, [0, 0, 0.15], p.getQuaternionFromEuler([0, 0, 0]))
# robot = p.loadURDF(_URDF, [0,0,0.15], p.getQuaternionFromEuler([0,0,math.pi/2]))
# p.changeDynamics(robot, -1, linearDamping=0.4, angularDamping=0.9)
p.changeDynamics(robot, -1, linearDamping=0.4, angularDamping=5.0)

jmap = {}
for i in range(p.getNumJoints(robot)):
    jmap[p.getJointInfo(robot, i)[1].decode()] = i

ARM = [jmap["arm_a"], jmap["arm_b"], jmap["arm_c"], jmap["arm_d"]]
FOOT = [jmap["foot_a"], jmap["foot_b"], jmap["foot_c"], jmap["foot_d"]]

for i in range(p.getNumJoints(robot)):
    p.changeDynamics(
        robot,
        i,
        lateralFriction=2.0,
        spinningFriction=0.1,
        rollingFriction=0.1,
        jointDamping=0.5,
    )

# ─── Debug UI ─────────────────────────────────────────────────────────────────
speed_id = p.addUserDebugParameter("Speed", 1, 10, 3)
step_btn_id = p.addUserDebugParameter("Step+1", 0, 100, 0)
status_id = p.addUserDebugText("Initializing...", [0, 0, 0.22], [1, 1, 1], 1.2)

# ─── State ────────────────────────────────────────────────────────────────────
cur_arm = list(LOG_STL)
cur_foot = [LOG_STL_F] * 4
paused = True
prev_step_btn = 0

KEY_SPACE = ord(" ")
KEY_R = ord("r")
KEY_Q = ord("q")
KEY_S = ord("s")
KEY_ESC = 27


def get_step_delay():
    return (11 - p.readUserDebugParameter(speed_id)) * 0.001


def apply_joints(step=None):
    for leg in range(4):
        if step is None:
            ar = arm_sim(leg, cur_arm[leg])
            fr = foot_sim(cur_foot[leg])
        else:
            ar = arm_sim(leg, crawl_arm_target(leg, step))
            fr = foot_sim(crawl_foot_target(leg, step))
        p.setJointMotorControl2(
            robot,
            ARM[leg],
            p.POSITION_CONTROL,
            targetPosition=ar,
            force=60,
            maxVelocity=3,
        )
        p.setJointMotorControl2(
            robot,
            FOOT[leg],
            p.POSITION_CONTROL,
            targetPosition=fr,
            # force=60,
            force=200,
            maxVelocity=3,
        )


def update_camera():
    pos, _ = p.getBasePositionAndOrientation(robot)
    cam = p.getDebugVisualizerCamera()
    dist, yaw, pitch = cam[10], cam[8], cam[9]
    p.resetDebugVisualizerCamera(dist, yaw, pitch, [pos[0], pos[1], 0.1])


# def stall():
#     for leg in range(4):
#         cur_arm[leg] = LOG_STL[leg]
#         cur_foot[leg] = LOG_STL_F
#     for _ in range(300):
#         apply_joints(step=None)
#         p.stepSimulation()
#         time.sleep(1.0 / PHYSICS_HZ)
#     p.resetBaseVelocity(robot, [0, 0, 0], [0, 0, 0])


def stall():
    # 정착하는 동안 몸체 위치 고정
    constraint = p.createConstraint(
        robot, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0.15]
    )
    for leg in range(4):
        cur_arm[leg] = LOG_STL[leg]
        cur_foot[leg] = LOG_STL_F
    for _ in range(300):
        apply_joints(step=None)
        p.stepSimulation()
        time.sleep(1.0 / PHYSICS_HZ)
    p.removeConstraint(constraint)  # 고정 해제
    p.resetBaseVelocity(robot, [0, 0, 0], [0, 0, 0])


# ─── Startup info ─────────────────────────────────────────────────────────────
LEG_NAMES = ["A(FL)", "B(FR)", "C(BL)", "D(BR)"]
_SWING_LEG_AT = {}
for _l in range(4):
    for _s in range(SWING_STEPS):
        _SWING_LEG_AT[(SWING_START[_l] + _s) % CRAWL_CYCLE] = _l

print("=" * 52)
print("  Spider Robot Walk Simulation (logical angle)")
print("=" * 52)
print("  Space=pause/run  S=step  R=restart  Q=quit")
print()
for i, n in enumerate("ABCD"):
    lo, hi = arm_log_range(i)
    print(
        f"  Leg {n}: stl={LOG_STL[i]:4d}  contact={LOG_CONTACT[i]:4d}"
        f"  toeoff={LOG_TOEOFF[i]:4d}  range=[{lo},{hi}]"
    )
print(f"  Foot: FDW={LOG_FDW} STL={LOG_STL_F} FUP={LOG_FUP} limit=±{FOOT_LIMIT}")
print()

stall()
# stall() 끝난 후 잠깐 테스트
# foot_a에 +0.5 rad 줘보기
p.setJointMotorControl2(
    robot, FOOT[0], p.POSITION_CONTROL, targetPosition=0.5, force=200, maxVelocity=1
)
for _ in range(100):
    p.stepSimulation()
    time.sleep(1.0 / PHYSICS_HZ)
# 발이 아래로 가면 양수=아래, 위로 가면 음수=아래

crawl_step = 0
cycle = 0
prev_swing = -1

# ─── Main loop ────────────────────────────────────────────────────────────────
try:
    while p.isConnected():
        keys = p.getKeyboardEvents()

        # Quit
        if (KEY_Q in keys and keys[KEY_Q] & p.KEY_WAS_TRIGGERED) or (
            KEY_ESC in keys and keys[KEY_ESC] & p.KEY_WAS_TRIGGERED
        ):
            break

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

        # Pause toggle
        if KEY_SPACE in keys and keys[KEY_SPACE] & p.KEY_WAS_TRIGGERED:
            paused = not paused
            print("[Space] Paused" if paused else "[Space] Running")

        # Paused: wait for step command
        if paused:
            s_key = KEY_S in keys and keys[KEY_S] & p.KEY_WAS_TRIGGERED
            btn_val = int(p.readUserDebugParameter(step_btn_id))
            slider_step = btn_val > prev_step_btn
            if slider_step:
                prev_step_btn = btn_val
            if not (s_key or slider_step):
                p.stepSimulation()
                time.sleep(0.016)
                continue

        # Apply gait
        apply_joints(step=crawl_step)

        if paused:
            for _ in range(30):
                p.stepSimulation()
        else:
            for _ in range(10):
                p.stepSimulation()
            time.sleep(get_step_delay())

        # Status
        sw = _SWING_LEG_AT.get(crawl_step, -1)
        swing_txt = f"swing={LEG_NAMES[sw]}" if sw != -1 else "stance"
        label = f"step={crawl_step:3d} cycle={cycle} {swing_txt}"
        if paused:
            label = "[P] " + label + " S=next Space=run"

        if sw != prev_swing and sw != -1:
            print(
                f"  step={crawl_step}: {LEG_NAMES[sw]} swinging"
                f"  contact={LOG_CONTACT[sw]}  toeoff={LOG_TOEOFF[sw]}"
            )
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
