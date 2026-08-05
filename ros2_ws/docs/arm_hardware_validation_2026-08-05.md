# Arm hardware validation — 2026-08-05

## Scope and safety state

This document records the four-axis arm mapping and minimum bidirectional
electrical motion checks performed on 2026-08-05. These checks do not complete
arm calibration and do not authorize Teleop, MoveIt, or FSM arm motion.

All individual tests ended with Torque Enable read back as OFF. No Hardware
Error was observed and no communication error occurred. The software safety
gates remain:

- `ARM_COMMAND_CALIBRATED=False`
- `ARM_LIMIT_ENABLED=False`

## Final motor mapping

| Joint | Hardware |
|---|---|
| `arm_joint_1` | No motor; fixed to the chassis |
| `arm_joint_2` | DYNAMIXEL ID 14 |
| `arm_joint_3` | DYNAMIXEL ID 13 |
| `arm_joint_4` | DYNAMIXEL ID 12 |
| `arm_joint_5` | DYNAMIXEL ID 16 |
| Gripper | DYNAMIXEL IDs 3 and 4 |

## Minimum bidirectional electrical motion checks

Only one arm motor was torque-enabled during each test. The other three arm
motors remained torque-off. Links were physically supported and a physical
power cut-off was available.

| Joint | ID | Positive test | Negative test | Result |
|---|---:|---:|---:|---|
| `arm_joint_2` | 14 | +5 tick command | -5 tick command | Encoder response in both directions |
| `arm_joint_3` | 13 | +10 tick command | -10 tick command | Encoder response in both directions |
| `arm_joint_4` | 12 | +10 tick command | -10 tick command | Encoder response in both directions |
| `arm_joint_5` | 16 | +20 tick command | -20 tick command | Encoder response in both directions |

Summary:

- Hardware Error was 0 throughout every test.
- No communication error occurred.
- Torque OFF was confirmed after every test.
- Small commands did not always reach the full commanded tick delta. Position
  tolerance, PID settings, load, friction, and profile values require later
  investigation.
- These tests confirm motor response and mapping, but do not establish the sign
  between increasing motor ticks and the URDF positive joint direction.

## Operating modes

| Joint | ID | Operating Mode |
|---|---:|---:|
| `arm_joint_2` | 14 | 4 — Extended Position Control |
| `arm_joint_3` | 13 | 4 — Extended Position Control |
| `arm_joint_4` | 12 | 3 — Position Control |
| `arm_joint_5` | 16 | 3 — Position Control |

The minimum motion tests used Profile Acceleration 1 and Profile Velocity 1.
These are test values, not production trajectory settings.

## ID 16 position warning

With ID 16 in Mode 3 and Torque OFF, Present Position was repeatedly observed
at approximately -34 ticks. Its modulo-4096 representation is approximately
4062 ticks:

```text
-34 mod 4096 = 4062
```

Earlier observations were approximately 680 ticks. Because the cause of this
difference is not established, neither -34 nor 4062 is a valid center value at
this time. Mode 3 position remapping or wrap behavior across Torque ON/OFF must
be tested while the physical pose is held fixed. Do not place either value in
`ARM_CENTERS` yet.

## Calibration items not complete

- `ARM_DIRECTIONS` is not determined.
- Mechanical center and software zero are not determined.
- Safe home and stow poses are not measured.
- Physical joint safety limits are not measured.
- `ARM_COMMAND_CALIBRATED` must remain `False`.
- `ARM_LIMIT_ENABLED` must remain `False`.
- Real arm control through Teleop, MoveIt, and the FSM is not validated.

The current encoder poses are temporary observations only. They must not be
treated as mechanical center, home, or stow values.

## Next work starting point

Proceed in this order:

1. Verify ID 16 Mode 3 position wrap/reinitialization while holding one fixed
   physical pose.
2. Compare increasing motor ticks for each joint with the URDF positive-axis
   rotation.
3. Set `ARM_DIRECTIONS` only after visual direction confirmation.
4. Define a reproducible center reference pose and measure it repeatedly.
5. Measure conservative physical joint safety limits.
6. Measure safe home and stow poses.
7. Only then update `ARM_CENTERS` and `ARM_DIRECTIONS` in code and review the
   command calibration gate.
8. Validate real control in the order Teleop, MoveIt, then FSM.

Until these steps are complete, do not set `ARM_COMMAND_CALIBRATED=True` and do
not enable full-arm command paths.
