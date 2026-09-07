"""Exercise the production switch with memory-only adapters, never a ROS/SDK node."""
import ast
from pathlib import Path
from types import SimpleNamespace

from dynamixel_control.tool_fsm.cleaner_fsm import CleanerFSM
from dynamixel_control.calibration_session import CalibrationSession
from dynamixel_control.dual_calibration_session import DualCalibrationSession
from dynamixel_control.dual_manual_recovery import DualManualRecovery
from dynamixel_control.tool_fsm.base import ToolState
from dynamixel_control.tool_manager import ToolManager, ParameterToolIdentityProvider
from dynamixel_control.tool_profiles import load_profiles, ToolProfileError


def test_bridge_mock_dual_spur_cleaner_dual_reuses_existing_contexts():
    source = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'
    tree = ast.parse(source.read_text())
    method = next(node for node in ast.walk(tree)
                  if isinstance(node, ast.FunctionDef) and node.name == '_switch_tool_runtime')
    namespace = dict(globals())
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    profile_path = Path(__file__).parents[1] / 'config/tool_profiles.yaml'
    bridge = SimpleNamespace(
        tool_type='cleaner', mock_mode=True, emergency_stop_active=False,
        tool_detached=False, _gripper_goal_active=False, tool_fsm=None,
        tool_ids=[], torque_enabled_ids=set(), active_ids=set(), tool_discovered=True,
        group_sync_read=SimpleNamespace(delParam=lambda _: None),
        get_parameter=lambda _: SimpleNamespace(value=str(profile_path)),
        get_logger=lambda: SimpleNamespace(info=lambda _: None))
    bridge.set_allowlist = lambda ids: setattr(bridge, '_fsm_allowlist', set(ids))
    bridge.read_position = lambda i: bridge._tool_samples[i]['position']
    bridge.read_torque = lambda _: 0
    bridge.read_hardware_error = lambda _: 0
    bridge.read_model = lambda _: 1060
    switch = namespace['_switch_tool_runtime']
    for tool, ids, fsm_name in (
            ('dual_motor_gripper', [3, 4], 'DualMotorGripperFSM'),
            ('spur_1motor_gripper', [5], 'SingleMotorGripperFSM'),
            ('cleaner', [], 'CleanerFSM'),
            ('dual_motor_gripper', [3, 4], 'DualMotorGripperFSM')):
        # The production switch requires the old tool to have stopped.
        if bridge.tool_fsm:
            bridge.tool_fsm.state = ToolState.STOPPED
        switch(bridge, tool)
        assert bridge.tool_type == tool
        assert bridge.tool_ids == bridge.tool_profile['actuator_ids'] == ids
        assert set(bridge._tool_samples) == set(ids)
        assert bridge._fsm_allowlist == set(ids)
        assert (type(bridge.tool_fsm).__name__ if bridge.tool_fsm else None) == fsm_name
        assert bool(bridge.calibration_session) == (tool == 'spur_1motor_gripper')
        assert bool(bridge.dual_calibration_session) == (tool == 'dual_motor_gripper')
        assert bool(bridge.dual_manual_recovery) == (tool == 'dual_motor_gripper')
