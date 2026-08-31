"""Read-only startup validation for tool FSMs."""


def validate_single_motor_startup(profile, bridge):
    """Return a snapshot after reads only; raise on an unsafe observation."""
    ids = profile.get('actuator_ids')
    if ids != [5]:
        raise ValueError('spur_1motor_gripper requires actuator_ids == [5]')
    # The bridge allowlist is policy state, not a register write.
    bridge.set_allowlist([5])
    model = bridge.read_model(5) if hasattr(bridge, 'read_model') else None
    position = bridge.read_position(5)
    torque = bridge.read_torque(5)
    hardware_error = bridge.read_hardware_error(5)
    if position is None or torque not in (0, 1):
        raise RuntimeError('ID5 feedback unavailable')
    if hardware_error != 0:
        raise RuntimeError(f'ID5 hardware error: {hardware_error}')
    expected = profile.get('motor_model')
    if expected is not None and model is not None and model != expected:
        raise RuntimeError(f'ID5 model {model!r} incompatible with {expected!r}')
    return {'id': 5, 'model': model, 'position': int(position),
            'torque': int(torque), 'hardware_error': int(hardware_error)}

