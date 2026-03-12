#!/usr/bin/env python3
"""
Spider Robot Walk Simulation - PyBullet
Reproduces WALK() gait from main.cpp

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

# ─── Parameters from main.cpp ────────────────────────────────────────────────
PHY_CONTACT = [100, 80, 30, 150]   # contact angle per leg [A, B, C, D]
ARM_DIR     = [1, -1, 1, -1]       # stance direction
STRIDE      = 50                    # stance range (deg)
PHY_STL     = [90, 90, 90, 90]     # neutral arm angle (deg)
FDW         = 30                    # foot down (standing)
FUP         = 60                    # foot up (lifted)

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

# ─── URDF ────────────────────────────────────────────────────────────────────
# Leg placement yaw angles (from +X axis, CCW):
#   A front-left  (-0.06, +0.06) : 135 deg
#   B front-right (+0.06, +0.06) :  45 deg
#   C rear-left   (-0.06, -0.06) : -135 deg
#   D rear-right  (+0.06, -0.06) :  -45 deg
URDF_STR = """<?xml version="1.0"?>
<robot name="spider">

  <link name="body">
    <visual>
      <geometry><box size="0.12 0.12 0.04"/></geometry>
      <material name="bd"><color rgba="0.25 0.45 0.85 1"/></material>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="1e-3" ixy="0" ixz="0" iyy="1e-3" iyz="0" izz="1e-3"/>
    </inertial>
  </link>

  <!-- Leg A : front-left (-0.06, +0.06), yaw=135 deg -->
  <joint name="arm_a" type="revolute">
    <parent link="body"/><child link="arm_a"/>
    <origin xyz="-0.06 0.06 0" rpy="0 0 2.3562"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5708" upper="1.5708" effort="10" velocity="5"/>
  </joint>
  <link name="arm_a">
    <visual><origin xyz="0.04 0 0" rpy="0 1.5708 0"/>
      <geometry><cylinder radius="0.007" length="0.08"/></geometry>
      <material name="g"><color rgba="0.55 0.55 0.55 1"/></material></visual>
    <inertial><mass value="0.05"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
  </link>
  <joint name="foot_a" type="revolute">
    <parent link="arm_a"/><child link="foot_a"/>
    <origin xyz="0.08 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="10" velocity="5"/>
  </joint>
  <link name="foot_a">
    <visual><origin xyz="0 0 -0.05"/>
      <geometry><cylinder radius="0.006" length="0.10"/></geometry>
      <material name="ra"><color rgba="0.9 0.2 0.2 1"/></material></visual>
    <inertial><mass value="0.03"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
  </link>

  <!-- Leg B : front-right (+0.06, +0.06), yaw=45 deg -->
  <joint name="arm_b" type="revolute">
    <parent link="body"/><child link="arm_b"/>
    <origin xyz="0.06 0.06 0" rpy="0 0 0.7854"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5708" upper="1.5708" effort="10" velocity="5"/>
  </joint>
  <link name="arm_b">
    <visual><origin xyz="0.04 0 0" rpy="0 1.5708 0"/>
      <geometry><cylinder radius="0.007" length="0.08"/></geometry>
      <material name="g"/></visual>
    <inertial><mass value="0.05"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
  </link>
  <joint name="foot_b" type="revolute">
    <parent link="arm_b"/><child link="foot_b"/>
    <origin xyz="0.08 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="10" velocity="5"/>
  </joint>
  <link name="foot_b">
    <visual><origin xyz="0 0 -0.05"/>
      <geometry><cylinder radius="0.006" length="0.10"/></geometry>
      <material name="rb"><color rgba="0.9 0.5 0.1 1"/></material></visual>
    <inertial><mass value="0.03"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
  </link>

  <!-- Leg C : rear-left (-0.06, -0.06), yaw=-135 deg -->
  <joint name="arm_c" type="revolute">
    <parent link="body"/><child link="arm_c"/>
    <origin xyz="-0.06 -0.06 0" rpy="0 0 -2.3562"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5708" upper="1.5708" effort="10" velocity="5"/>
  </joint>
  <link name="arm_c">
    <visual><origin xyz="0.04 0 0" rpy="0 1.5708 0"/>
      <geometry><cylinder radius="0.007" length="0.08"/></geometry>
      <material name="g"/></visual>
    <inertial><mass value="0.05"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
  </link>
  <joint name="foot_c" type="revolute">
    <parent link="arm_c"/><child link="foot_c"/>
    <origin xyz="0.08 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="10" velocity="5"/>
  </joint>
  <link name="foot_c">
    <visual><origin xyz="0 0 -0.05"/>
      <geometry><cylinder radius="0.006" length="0.10"/></geometry>
      <material name="rc"><color rgba="0.2 0.8 0.3 1"/></material></visual>
    <inertial><mass value="0.03"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
  </link>

  <!-- Leg D : rear-right (+0.06, -0.06), yaw=-45 deg -->
  <joint name="arm_d" type="revolute">
    <parent link="body"/><child link="arm_d"/>
    <origin xyz="0.06 -0.06 0" rpy="0 0 -0.7854"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5708" upper="1.5708" effort="10" velocity="5"/>
  </joint>
  <link name="arm_d">
    <visual><origin xyz="0.04 0 0" rpy="0 1.5708 0"/>
      <geometry><cylinder radius="0.007" length="0.08"/></geometry>
      <material name="g"/></visual>
    <inertial><mass value="0.05"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
  </link>
  <joint name="foot_d" type="revolute">
    <parent link="arm_d"/><child link="foot_d"/>
    <origin xyz="0.08 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="10" velocity="5"/>
  </joint>
  <link name="foot_d">
    <visual><origin xyz="0 0 -0.05"/>
      <geometry><cylinder radius="0.006" length="0.10"/></geometry>
      <material name="rd"><color rgba="0.3 0.5 0.9 1"/></material></visual>
    <inertial><mass value="0.03"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
  </link>

</robot>
"""

# ─── PyBullet setup ──────────────────────────────────────────────────────────
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.resetDebugVisualizerCamera(
    cameraDistance=0.5,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.1]
)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

p.loadURDF("plane.urdf")

tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
tmp.write(URDF_STR)
tmp.close()
try:
    robot = p.loadURDF(tmp.name, [0, 0, 0.12], p.getQuaternionFromEuler([0, 0, 0]))
finally:
    os.unlink(tmp.name)

# ─── Joint index map ─────────────────────────────────────────────────────────
jmap = {}
for i in range(p.getNumJoints(robot)):
    name = p.getJointInfo(robot, i)[1].decode()
    jmap[name] = i

ARM  = [jmap['arm_a'],  jmap['arm_b'],  jmap['arm_c'],  jmap['arm_d']]
FOOT = [jmap['foot_a'], jmap['foot_b'], jmap['foot_c'], jmap['foot_d']]

# Fix body in place (visualization only - show gait pattern)
p.createConstraint(robot, -1, -1, -1, p.JOINT_FIXED, [0,0,0], [0,0,0], [0,0,0.12])

# ─── Debug UI ────────────────────────────────────────────────────────────────
speed_id = p.addUserDebugParameter("Speed (1=slow, 10=fast)", 1, 10, 5)

# Status text (top-left)
status_id = p.addUserDebugText("Initializing...", [0, 0, 0.22], [1,1,1], 1.2)

# ─── State ───────────────────────────────────────────────────────────────────
cur_arm  = list(PHY_STL)
cur_foot = [FDW, FDW, FDW, FDW]
paused   = False

KEY_SPACE = ord(' ')
KEY_R     = ord('r')
KEY_Q     = ord('q')
KEY_ESC   = 27

def get_step_delay():
    speed = p.readUserDebugParameter(speed_id)   # 1..10
    return (11 - speed) * 0.001                   # 10ms..1ms

def apply_joints():
    for leg in range(4):
        p.setJointMotorControl2(robot, ARM[leg],  p.POSITION_CONTROL,
                                targetPosition=arm_sim(leg, cur_arm[leg]),
                                force=10, maxVelocity=5)
        p.setJointMotorControl2(robot, FOOT[leg], p.POSITION_CONTROL,
                                targetPosition=foot_sim(cur_foot[leg]),
                                force=10, maxVelocity=5)

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
            p.addUserDebugText(label, [0, 0, 0.22], [1, 0.4, 0.4], 1.2,
                               replaceItemUniqueId=status_id)
        else:
            print("[Space] Resumed")

    # Quit
    if (KEY_Q in keys and keys[KEY_Q] & p.KEY_WAS_TRIGGERED) or \
       (KEY_ESC in keys and keys[KEY_ESC] & p.KEY_WAS_TRIGGERED):
        return False

    if paused:
        apply_joints()
        p.stepSimulation()
        time.sleep(0.016)
        return True

    apply_joints()
    p.stepSimulation()
    time.sleep(get_step_delay())
    return True

def wait_for_resume():
    """Spin until unpaused or quit."""
    while p.isConnected():
        if not sim_step():
            return False
        if not paused:
            return True
    return False

# ─── Motion primitives (mirror of main.cpp) ──────────────────────────────────
def arm_to(leg, target):
    cur = cur_arm[leg]
    step = 1 if target > cur else -1
    for a in range(cur, target + step, step):
        cur_arm[leg] = a
        if not sim_step():
            raise SystemExit
        if paused and not wait_for_resume():
            raise SystemExit

def foot_up(leg):
    for f in range(FDW, FUP + 1):
        cur_foot[leg] = f
        if not sim_step():
            raise SystemExit
        if paused and not wait_for_resume():
            raise SystemExit

def foot_dw(leg):
    for f in range(FUP, FDW - 1, -1):
        cur_foot[leg] = f
        if not sim_step():
            raise SystemExit
        if paused and not wait_for_resume():
            raise SystemExit

def stall():
    for leg in range(4):
        cur_arm[leg]  = PHY_STL[leg]
        cur_foot[leg] = FDW
    for _ in range(60):
        apply_joints()
        p.stepSimulation()
        time.sleep(0.003)

LEG_NAMES = ['A(FL)', 'B(FR)', 'C(RL)', 'D(RR)']

def WALK():
    """Exact reproduction of main.cpp WALK(): A -> D -> B -> C"""
    order = [0, 3, 1, 2]
    for leg in order:
        label = f"Leg {LEG_NAMES[leg]} swinging  contact={PHY_CONTACT[leg]}  toeoff={phy_toeoff(leg)}"
        print(f"  {label}")
        p.addUserDebugText(label, [0, 0, 0.22], [0.3, 1.0, 0.4], 1.2,
                           replaceItemUniqueId=status_id)
        foot_up(leg)
        arm_to(leg, PHY_CONTACT[leg])
        foot_dw(leg)
        arm_to(leg, phy_toeoff(leg))

# ─── Main ────────────────────────────────────────────────────────────────────
print("=" * 52)
print("  Spider Robot Walk Simulation")
print("=" * 52)
print("  Camera : left-drag=rotate  right-drag=pan  scroll=zoom")
print("  Keys   : [Space]=pause/resume  [R]=restart  [Q]=quit")
print()
print("  Gait parameters:")
for i, n in enumerate('ABCD'):
    print(f"    Leg {n}: contact={PHY_CONTACT[i]:3d} deg  "
          f"toeoff={phy_toeoff(i):3d} deg  "
          f"dir={'CW' if ARM_DIR[i]==1 else 'CCW'}")
print()

stall()
time.sleep(0.5)

cycle = 0
try:
    while p.isConnected():
        # Check R key for restart
        keys = p.getKeyboardEvents()
        if KEY_R in keys and keys[KEY_R] & p.KEY_WAS_TRIGGERED:
            print("[R] Restarting...")
            stall()
            cycle = 0
            time.sleep(0.3)
            continue

        cycle += 1
        print(f"=== Cycle {cycle} ===")
        p.addUserDebugText(f"Cycle {cycle}", [0, 0, 0.22], [1,1,1], 1.2,
                           replaceItemUniqueId=status_id)
        WALK()
        time.sleep(0.3)

except (SystemExit, KeyboardInterrupt):
    pass

if p.isConnected():
    p.disconnect()
print("Simulation ended.")
