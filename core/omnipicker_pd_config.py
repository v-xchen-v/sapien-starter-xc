"""
PD gain configuration for Agibot OmniPicker robot with mimic joints.

This module provides utilities to configure drive properties (PD gains) for
the OmniPicker gripper, which uses mimic joints to simulate coordinated finger movement.
"""

import sapien.core as sapien
from typing import Dict, Optional


# Mimic joint multipliers for single-arm OmniPicker
OMNIPICKER_MIMIC_MULTIPLIERS = {
    'narrow2_joint': 0.02,
    'narrow3_joint': 0.4,
    'narrow_loop_joint': 1.5,
    'wide1_joint': 1.0,
    'wide2_joint': 0.02,
    'wide3_joint': 0.4,
    'wide_loop_joint': 1.5,
}


def get_dual_arm_mimic_multipliers() -> Dict[str, float]:
    """
    Generate mimic multipliers for dual-arm OmniPicker configuration.
    
    Returns:
        Dictionary mapping joint names to their mimic multipliers for both left and right arms.
    """
    dual_arm_multipliers = {}
    
    # Add left arm mimic joints
    for key, value in OMNIPICKER_MIMIC_MULTIPLIERS.items():
        dual_arm_multipliers[f'left_{key}'] = value
    
    # Add right arm mimic joints
    for key, value in OMNIPICKER_MIMIC_MULTIPLIERS.items():
        dual_arm_multipliers[f'right_{key}'] = value
    
    return dual_arm_multipliers


def set_g1_omnipicker_drive_properties(
    robot,
    gripper_mimic_multipliers: Dict[str, float],
    gripper_mimic_stiffness: float = 1000.0,
    gripper_mimic_damping: float = 0.0,
    gripper_master_stiffness: float = 2000.0,
    gripper_master_damping: float = 100.0,
    arm_stiffness: float = 2000.0,
    arm_damping: float = 100.0,
    arm_force_limit: Optional[float] = None
) -> None:
    """
    Set drive properties for OmniPicker robot joints.
    
    For mimic joints (gripper fingers), high stiffness with zero damping is used
    to prevent shaking and ensure coordinated movement. For arm joints, normal
    PD control parameters are applied.
    
    Args:
        robot: The SAPIEN articulation (robot) to configure
        mimic_multipliers: Dictionary mapping mimic joint names to their multipliers
        mimic_stiffness: Stiffness for mimic joints (default: 1000.0)
        mimic_damping: Damping for mimic joints (default: 0.0)
        arm_stiffness: Stiffness for arm joints (default: 2000.0)
        arm_damping: Damping for arm joints (default: 100.0)
        arm_force_limit: Force limit for arm joints (default: None)
    
    Notes:
        - Mimic joints use high stiffness and zero damping to avoid shaking
        - Arm joints use standard PD control with moderate stiffness and damping
        - Force limits can be set for arm joints to prevent excessive forces
    """
    for joint in robot.get_active_joints():
        joint_name = joint.get_name()
        
        if joint_name in gripper_mimic_multipliers:
            # Mimic joints: high stiffness, zero damping
            joint.set_drive_properties(stiffness=gripper_mimic_stiffness, damping=gripper_mimic_damping)
        elif "gripper_joint" in joint_name:
            # Master gripper joint
            joint.set_drive_properties(
                stiffness=gripper_master_stiffness,
                damping=gripper_master_damping,
                force_limit=200.0
            )
        else:
            # Arm joints: normal PD control
            if arm_force_limit is not None:
                joint.set_drive_properties(
                    stiffness=arm_stiffness,
                    damping=arm_damping,
                    force_limit=arm_force_limit
                )
            else:
                joint.set_drive_properties(stiffness=arm_stiffness, damping=arm_damping)
                
            # Extra: set armature to improve arm stability
            joint.set_armature([0.5])


def initialize_omnipicker_qpos(
    robot,
    mimic_multipliers: Dict[str, float],
    gripper_value: float = 0.0
) -> None:
    """
    Initialize joint positions for OmniPicker robot with mimic joints.
    
    Args:
        robot: The SAPIEN articulation (robot) to configure
        mimic_multipliers: Dictionary mapping mimic joint names to their multipliers
        gripper_value: Initial gripper opening value (0=closed, 1=open)
    """
    qpos = []
    for joint in robot.get_active_joints():
        joint_name = joint.get_name()
        if joint_name in mimic_multipliers:
            qpos.append(gripper_value * mimic_multipliers[joint_name])
            joint.set_drive_target(gripper_value * mimic_multipliers[joint_name])
        elif "gripper_joint" in joint_name:
            qpos.append(0.0)
            joint.set_drive_target(0.0)
    
    # robot.set_qpos(qpos)



# Preset configurations for common use cases
class PDConfig:
    """Preset PD configurations for different control scenarios."""
    
    # Configuration 1: Stable gripper control (recommended)
    STABLE_G1_GRIPPER = {
        'gripper_mimic_stiffness': 1000.0,
        'gripper_mimic_damping': 0.0,
        'gripper_master_stiffness': 100.0,
        'gripper_master_damping': 10.0,
        'arm_stiffness': 2000.0,
        'arm_damping': 100.0,
    }
    
    # # Configuration 2: High stiffness for precise control
    # HIGH_STIFFNESS = {
    #     'mimic_stiffness': 2000.0,
    #     'mimic_damping': 0.0,
    #     'arm_stiffness': 100.0,
    #     'arm_damping': 10.0,
    #     'arm_force_limit': 50.0,
    # }
    
    # # Configuration 3: Soft control for compliance
    # SOFT_CONTROL = {
    #     'mimic_stiffness': 500.0,
    #     'mimic_damping': 0.0,
    #     'arm_stiffness': 100.0,
    #     'arm_damping': 10.0,
    # }


# Example usage
if __name__ == "__main__":
    print("OmniPicker PD Configuration Module")
    print(f"Single-arm mimic joints: {list(OMNIPICKER_MIMIC_MULTIPLIERS.keys())}")
    print(f"Dual-arm mimic joints: {len(get_dual_arm_mimic_multipliers())} joints")
    # print(f"\nAvailable presets: {', '.join(PDConfig.__dict__.keys() if not k.startswith('_') else '' for k in PDConfig.__dict__.keys())}")
