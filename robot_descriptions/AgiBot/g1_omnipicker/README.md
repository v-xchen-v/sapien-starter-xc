# Agibot G1 with Gripper Description

![Agibot G1 with Omni-Picker](../.images/agibot_g1_with_omnipicker.png)

## Available URDFs
- **agibot_g1_with_omnipicker.urdf** - G1 robot with Omni-Picker grippers (documented below)
- **agibot_g1_with_120s.urdf** - G1 robot with 120S grippers (TBD)

---

## G1 with Omni-Picker Configuration

### Important Links
- `left_end_link`
- `right_end_link`

The end of left/right arm, used as the arm end for IK/motion planning target link.

### Important Joints
- Arm Joints (7 DOF per arm)
  - **Left arm**: `left_joint1`, `left_joint2`, `left_joint3`, `left_joint4`, `left_joint5`, `left_joint6`, `left_joint7`
  - **Right arm**: `right_joint1`, `right_joint2`, `right_joint3`, `right_joint4`, `right_joint5`, `right_joint6`, `right_joint7`

- Gripper Joints (1 primary joint per gripper)
  - **Left gripper**: `left_gripper_joint` (revolute, controls all gripper fingers via mimic joints)
  - **Right gripper**: `right_gripper_joint` (revolute, controls all gripper fingers via mimic joints)

### URDF Structure
There are three important parts in this URDF:

1. G1 base to arm end
    The main robot body and arm chain
    - **Links**: `base_link` → `body_link1` → `body_link2` → `left_base_link`/`right_base_link` → ... → `arm_l_end_link`/`arm_r_end_link`
    - **Key end links**: `arm_l_end_link`, `arm_r_end_link`

2. Arm end to flange (camera stand end)
    The connection between arm end and the flange/camera mount
    - **Left arm**:
      - **Links**: `arm_l_end_link` → `left_camera_stand` → `left_camera_stand_end`
      - **Joints**: `left_camera_stand_base_joint` (fixed joint with 1/2 flange thickness for camera stand visual), `left_camera_stand_mount_joint` (fixed joint with flange thickness for gripper mount)
    - **Right arm**:
      - **Links**: `arm_r_end_link` → `right_camera_stand` → `right_camera_stand_end`
      - **Joints**: `right_camera_stand_joint` (fixed joint with 1/2 flange thickness for camera stand visual), `right_camera_stand_mount_joint` (fixed joint with flange thickness for gripper mount)

3. Flange end to gripper base
    The attachment from flange to the gripper base
    - **Left arm**:
      - **Links**: `left_camera_stand_end` → `gripper_l_base_link`
      - **Joint**: `left_gripper_mount` (fixed joint with 90° yaw rotation)
    - **Right arm**:
      - **Links**: `right_camera_stand_end` → `gripper_r_base_link`
      - **Joint**: `right_gripper_mount` (fixed joint with 90° yaw rotation)
