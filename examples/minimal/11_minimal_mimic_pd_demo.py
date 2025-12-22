"""
minimal_mimic_pd_demo.py

A minimal example to demonstrate position control with mimic joints on a robot using SAPIEN.

What you get:
- Load a robot URDF with mimic joints
- Sets PD drive (stiffness/damping) for both regular and mimic joints
- In the sim loop, enforces mimic by updating the mimic joint's PD target each step

Run:
    python examples/minimal/11_minimal_mimic_pd_demo.py
    

Notes:
- Sapien does not automatically handle mimic joints for you in PD control. We parse URDF ourselves and apply coupling each step.
- This demo uses a tiny "gripper-like" two-finger machinism (2 revolute joints)
"""

import os
import time
import math
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import sapien.core as sapien

URDF_TEXT = r"""
<?xml version="1.0"?>
<robot name="mimic_demo">
  <link name="base"/>

  <link name="left_finger_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/>
    </inertial>
  </link>

  <link name="right_finger_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/>
    </inertial>
  </link>

  <!-- Left finger joint (master) -->
  <joint name="left_finger_joint" type="revolute">
    <parent link="base"/>
    <child link="left_finger_link"/>
    <origin xyz="0 0.03 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="20" velocity="2.0"/>
  </joint>

  <!-- Right finger joint (mimic) -->
  <joint name="right_finger_joint" type="revolute">
    <parent link="base"/>
    <child link="right_finger_link"/>
    <origin xyz="0 -0.03 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="20" velocity="2.0"/>
    <!-- mimic: right = (-1) * left + 0 -->
    <mimic joint="left_finger_joint" multiplier="-1.0" offset="0.0"/>
  </joint>
</robot>
""".strip()


def parse_mimic_map_from_urdf(urdf_path: str):
    """
    Returns:
        mimic_map: dict[mimic_joint_name] = (master_joint_name, multiplier, offset)
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    mimic_map = {}
    for joint in root.findall("joint"):
        jname = joint.get("name")
        mimic = joint.find("mimic")
        if mimic is None:
            continue
        master = mimic.get("joint")
        mult = float(mimic.get("multiplier", "1.0"))
        offs = float(mimic.get("offset", "0.0"))
        mimic_map[jname] = (master, mult, offs)
    return mimic_map


def generate_mimic_pairs(mimic_map: dict, joints: list):
    """
    Generate mimic pairs from the mimic map, converting names to joint objects and indices.
    
    Args:
        mimic_map: dict[mimic_joint_name] = (master_joint_name, multiplier, offset)
        joints: List of joint objects from the robot
        
    Returns:
        List of tuples (mimic_joint, master_joint, mimic_qidx, master_qidx, mult, offs)
        where each element contains joint objects and their corresponding indices
    """
    # Build name -> joint and name -> qpos index mappings
    name_to_joint = {j.get_name(): j for j in joints}
    name_to_qidx = {}
    cursor = 0
    for j in joints:
        dof = int(j.get_dof())
        if dof > 0:
            name_to_qidx[j.get_name()] = cursor
            cursor += dof
    
    mimic_pairs = []
    for mimic_jname, (master_jname, mult, offs) in mimic_map.items():
        if mimic_jname not in name_to_qidx or master_jname not in name_to_qidx:
            print(f"[WARN] mimic pair skipped (no dof index): {mimic_jname} <- {master_jname}")
            continue
        mimic_pairs.append(
            (name_to_joint[mimic_jname], name_to_joint[master_jname], 
             name_to_qidx[mimic_jname], name_to_qidx[master_jname], mult, offs)
        )
    return mimic_pairs


def set_master_joint_with_mimic(
    master_joint,  # joint object
    master_target: float,
    mimic_pairs: list,
):
    """
    Set the master joint's PD target and automatically apply mimic mapping rules to all mimic joints.
    
    Args:
        master_joint: The master joint object to control
        master_target: Target position (qpos) for the master joint
        mimic_pairs: List of tuples (mimic_joint, master_joint, mimic_qidx, master_qidx, mult, offs)
                     where mimic_target = mult * master_target + offs
                     Note: mimic_joint and master_joint should be joint objects
    """
    # Set the master joint target
    master_joint.set_drive_target(master_target)
    
    # Apply mimic rules: mimic_target = mult * master_target + offset
    for mimic_joint, master_jnt, mimic_qidx, master_qidx, mult, offs in mimic_pairs:
        # Only apply if this mimic joint is controlled by our master joint
        if master_jnt == master_joint:
            mimic_target = mult * master_target + offs
            mimic_joint.set_drive_target(float(mimic_target))


def main():
    # 1) Write URDF to a temp file
    tmp_dir = tempfile.mkdtemp(prefix="sapien_mimic_demo_")
    urdf_path = os.path.join(tmp_dir, "mimic_demo.urdf")
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write(URDF_TEXT)
    print(f"[INFO] Wrote URDF to: {urdf_path}")

    # 2) Parse mimic mapping (because SAPIEN doesn't enforce it for you)
    mimic_map = parse_mimic_map_from_urdf(urdf_path)
    print("[INFO] mimic_map =", mimic_map)

    # 3) Create engine/scene
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)

    scene = engine.create_scene()
    scene.set_timestep(1.0 / 240.0)
    scene.add_ground(altitude=0.0)

    # 4) Load articulation
    loader = scene.create_urdf_loader()
    # Keep root fixed (otherwise you get a floating base)
    loader.fix_root_link = True

    robot = loader.load(urdf_path)
    if robot is None:
        raise RuntimeError("Failed to load URDF into SAPIEN.")

    robot.set_name("mimic_demo_robot")

    # 5) Get joints
    joints = robot.get_joints()

    # 6) Set PD drives for joints (including mimic joint)
    # Tune these as you like:
    stiffness = 200.0
    damping = 20.0
    force_limit = 50.0

    # Apply to all active joints (those with DoF > 0)
    qpos = robot.get_qpos()
    qpos_idx = 0
    for j in joints:
        dof = int(j.get_dof())
        if dof > 0:
            # Drive mode default is position-based PD in many builds; we set properties explicitly.
            # Some SAPIEN versions have signature: set_drive_property(stiffness, damping, force_limit, mode)
            # Others: set_drive_property(stiffness, damping, force_limit)
            try:
                j.set_drive_property(stiffness, damping, force_limit)
            except TypeError:
                # Fallback for older signature
                j.set_drive_property(stiffness, damping, force_limit, "force")
            # Initialize targets at current position to avoid impulse
            j.set_drive_target(float(qpos[qpos_idx]))
            qpos_idx += dof

    # 7) Convenience: precompute mimic joint indices and master indices
    mimic_pairs = generate_mimic_pairs(mimic_map, joints)

    active_joint_names = [j.get_name() for j in joints if j.get_dof() > 0]
    print("[INFO] Active 1-DoF joints:", active_joint_names)
    print("[INFO] Mimic pairs resolved:", mimic_pairs)

    # 8) Simulation loop: command master joint target, then enforce mimic target each step
    # We'll drive left_finger_joint with a sinusoid, and right_finger_joint follows via mimic.
    master_name = "left_finger_joint"
    name_to_joint = {j.get_name(): j for j in joints}
    
    if master_name not in name_to_joint:
        raise RuntimeError(f"Expected joint '{master_name}' not found. Joints: {list(name_to_joint.keys())}")

    master_joint = name_to_joint[master_name]

    # For printing, build qpos index mapping
    name_to_qidx = {}
    cursor = 0
    for j in joints:
        dof = int(j.get_dof())
        if dof > 0:
            name_to_qidx[j.get_name()] = cursor
            cursor += dof

    steps = 240 * 5  # 5 seconds
    t0 = time.time()
    for k in range(steps):
        t = (k / 240.0)
        # Master target (radians)
        master_target = 0.6 * math.sin(2.0 * math.pi * 0.5 * t)  # 0.5 Hz

        # Set master joint and apply mimic rules using the extracted method
        set_master_joint_with_mimic(master_joint, master_target, mimic_pairs)

        scene.step()

        # Print occasionally
        if k % 120 == 0:
            qpos = robot.get_qpos()
            left_q = qpos[name_to_qidx["left_finger_joint"]]
            right_q = qpos[name_to_qidx["right_finger_joint"]]
            print(
                f"t={t:5.2f}s  target(left)={master_target:+.3f}  "
                f"qpos(left)={left_q:+.3f}  qpos(right)={right_q:+.3f}  "
                f"(expected right≈{-left_q:+.3f})"
            )

    print(f"[DONE] Simulated {steps} steps in {time.time() - t0:.2f}s")


if __name__ == "__main__":
    main()