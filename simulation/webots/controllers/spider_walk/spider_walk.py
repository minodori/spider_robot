"""
Spider Robot - Webots Controller
Ported from PyBullet simulation code

Legs: A(front-left), B(front-right), C(rear-left), D(rear-right)
Gait order: A -> D -> B -> C (static creep)
"""

from controller import Robot
import math

# ─── Parameters (from main.cpp) ──────────────────────────────────────────────
PHY_STL = [90, 90, 90, 90]  # neutral arm angle (deg)
FDW = 30  # foot down (standing)
FUP = 60  # foot up (lifted)
ARM_DIR = [1, -1, 1, -1]  # stance direction per leg
STRIDE = 50  # stance range (deg)
PHY_CONTACT = [100, 80, 30, 150]  # contact angle per leg [A,B,C,D]


def phy_toeoff(leg):
    return PHY_CONTACT[leg] + ARM_DIR[leg] * STRIDE


# ─── Angle conversion ────────────────────────────────────────────────────────
def arm_rad(leg, phy_deg):
    """Physical servo angle (deg) → Webots joint angle (rad)"""
    return math.radians(phy_deg - PHY_STL[leg])


def foot_rad(phy_deg):
    """Foot servo angle (deg) → Webots joint angle (rad)"""
    return -math.radians(phy_deg - FDW)


# ─── Robot init ──────────────────────────────────────────────────────────────
robot = Robot()
timestep = int(robot.getBasicTimeStep())  # ms

# Motor 이름은 URDF joint 이름과 동일
ARM_NAMES = ["arm_a", "arm_b", "arm_c", "arm_d"]
FOOT_NAMES = ["foot_a", "foot_b", "foot_c", "foot_d"]

arms = [robot.getDevice(n) for n in ARM_NAMES]
foots = [robot.getDevice(n) for n in FOOT_NAMES]

# 속도 설정
for m in arms + foots:
    m.setVelocity(2.0)

# 현재 각도 상태
cur_arm = list(PHY_STL)
cur_foot = [FDW, FDW, FDW, FDW]


# ─── Primitives ──────────────────────────────────────────────────────────────
def apply_joints():
    for i in range(4):
        arms[i].setPosition(arm_rad(i, cur_arm[i]))
        foots[i].setPosition(foot_rad(cur_foot[i]))


def step(n=1):
    """Webots simulation step"""
    for _ in range(n):
        if robot.step(timestep) == -1:
            return False
    return True


def arm_to(leg, target):
    cur = cur_arm[leg]
    inc = 1 if target > cur else -1
    for a in range(cur, target + inc, inc):
        cur_arm[leg] = a
        apply_joints()
        if not step():
            return False
    return True


def foot_up(leg):
    for f in range(FDW, FUP + 1):
        cur_foot[leg] = f
        apply_joints()
        if not step():
            return False
    return True


def foot_dw(leg):
    for f in range(FUP, FDW - 1, -1):
        cur_foot[leg] = f
        apply_joints()
        if not step():
            return False
    return True


def stall():
    """초기 자세로 복귀"""
    for i in range(4):
        cur_arm[i] = PHY_STL[i]
        cur_foot[i] = FDW
    apply_joints()
    step(30)


# ─── Gait ────────────────────────────────────────────────────────────────────
LEG_NAMES = ["A(FL)", "B(FR)", "C(RL)", "D(RR)"]


def WALK():
    """Static creep: A → D → B → C"""
    order = [0, 3, 1, 2]
    for leg in order:
        print(f"  Leg {LEG_NAMES[leg]} swinging")
        if not foot_up(leg):
            return False
        if not arm_to(leg, PHY_CONTACT[leg]):
            return False
        if not foot_dw(leg):
            return False
        if not arm_to(leg, phy_toeoff(leg)):
            return False
    return True


# ─── Main loop ───────────────────────────────────────────────────────────────
print("=== Spider Walk Controller (Webots) ===")
print(f"  timestep: {timestep}ms")

stall()

cycle = 0
while robot.step(timestep) != -1:
    cycle += 1
    print(f"=== Cycle {cycle} ===")
    if not WALK():
        break
