"""Compatibility shell for the existing dual-motor gripper policy.

The old bridge implementation remains its execution adapter during this first
split.  This class deliberately does not claim ID5 behaviour.
"""

from dynamixel_control.tool_fsm.base import ToolFSM


class DualMotorGripperFSM(ToolFSM):
    TOOL_TYPE = 'dual_motor_gripper'

    def startup(self):
        self.bridge.set_allowlist(self.actuator_ids)
        return self.state

