"""Shared configuration for the four actuated arm joints.

``arm_joint_1`` is the unpowered chassis-to-arm yaw connection.  It remains
in the robot model as a fixed transform, but is deliberately absent here.
"""

import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET


ARM_JOINT_NAMES = [f"arm_joint_{index}" for index in range(2, 6)]

# Read-only manual encoder mapping, 2026-08-05.
ARM_MOTOR_IDS = [14, 13, 12, 16]

# TODO(HW calibration): these are neutral conversion placeholders retained so
# read-only joint-state diagnostics can express a provisional radian value.
# They are NOT measured zero offsets/directions and must not authorize writes.
ARM_CENTERS = [2048] * 4
ARM_DIRECTIONS = [1] * 4
ARM_COMMAND_CALIBRATED = False

# arm_joint_1 is physically fixed to the chassis.  The URDF currently preserves
# the CAD-zero transform only; the installed chassis yaw has not been measured.
ARM_FIXED_YAW_CALIBRATED = False

ARM_JOINT_CONFIG = {
    name: {"id": ARM_MOTOR_IDS[index],
           "center": ARM_CENTERS[index],
           "direction": ARM_DIRECTIONS[index]}
    for index, name in enumerate(ARM_JOINT_NAMES)
}

# TODO(HW calibration): URDF/CAD limits, not validated physical safe limits.
# Keep software limiting disabled until center, direction, and limits are
# measured.  Hardware command startup is blocked by ARM_COMMAND_CALIBRATED.
ARM_LIMIT_ENABLED = [False] * 4
ARM_MIN_RADS = [0.0, 0.0, -0.610865, -math.pi]
ARM_MAX_RADS = [math.pi, math.pi, 0.698132, math.pi]


def load_srdf_group_state(group_name, state_name):
    """Load a complete named joint state from the installed MoveIt SRDF."""
    candidates = [
        Path(prefix) / "share/robot_arm_moveit_config/config/robot_arm.srdf"
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        if prefix
    ]
    path = next((candidate for candidate in candidates if candidate.is_file()), None)
    if path is None:
        raise RuntimeError(
            "robot_arm_moveit_config/config/robot_arm.srdf is not installed "
            "in AMENT_PREFIX_PATH")
    root = ET.parse(path).getroot()
    state = root.find(
        f"./group_state[@name='{state_name}'][@group='{group_name}']")
    if state is None:
        raise RuntimeError(
            f"SRDF group_state {group_name}/{state_name} was not found")
    values = {
        joint.get("name"): float(joint.get("value"))
        for joint in state.findall("joint")
    }
    missing = [name for name in ARM_JOINT_NAMES if name not in values]
    if missing:
        raise RuntimeError(
            f"SRDF group_state {group_name}/{state_name} misses {missing}")
    return [values[name] for name in ARM_JOINT_NAMES]
