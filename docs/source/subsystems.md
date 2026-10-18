# Subsystems

`TankDrive` owns drive mixing, voltage limits, disabled protection, and logging. `TankDriveIO` separates real SPARK MAX control, simulation, and replay. `TankTeleopCommand` reads a USB 0 Xbox controller.

Shooter uses `Flywheel`/`FlywheelIO`, pivot uses `PositionJoint`/`PositionJointIO`, and intake/indexer use open-loop `Motor`/`MotorIO`. The closed-loop classes follow the rebuilt gain records and `LoggedTunableNumber.ifChanged` updates, with controller-side SPARK MAX PID/MAXMotion in real IO. Shooter feedback uses mechanism rotations/second, matching the rebuilt flywheel; pivot uses absolute mechanism rotations and rotations/second. Named factories select real/simulation/replay and keep vendor calls out of subsystems. Hardware records and gains are in `constants/subsystems`.

Pivot startup assumes physical home at +90 degrees (+0.25 rotations). Horizontal is 0 degrees; positive `kG*cos(angle)` commands increasing angle toward stow. Deployed is -60 degrees and intermediate is +15 degrees. Limits allow -60 to +90 degrees. Reference status is diagnostic and does not gate motion. The optional disabled home-reference command can restore the angle after a relative-encoder reset. Disconnected or disabled outputs remain neutral.
