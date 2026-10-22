# Architecture

`Robot` starts AdvantageKit and runs the command scheduler. `RobotContainer` declares tank drive, the USB 0 Xbox controller, the default drive command, hub vision, and the autonomous chooser.

`TankTeleopCommand` reads controller input in enabled teleop. `TankDrive` shapes inputs, mixes voltages, stops on disable, and logs IO. `TankDriveIO` selects real SPARK MAX control, differential drivetrain simulation, or replay.

CAN 1 is left drive; CAN 2 is right drive. Real wheel distance and heading are uncalibrated. `HubVisionIO` isolates the PhotonVision adapter from the Y-button `DistanceBasedShooterCommand`. The current disconnected adapter keeps the shooter stopped until the camera integration supplies fresh, low-ambiguity observations of 2026 hub tags. The command interpolates between three measured distance/RPS points and does not require the drivetrain, so manual driving remains available. `DistanceShooterCalibrationStore` loads a validated table from WPILib Preferences and saves live tunables only after an explicit disabled-mode request.

The container declares `Flywheel` for shooter, `PositionJoint` for pivot, and open-loop `Motor` instances for intake/indexer. Subsystems own targets and logging. Like the rebuilt project, tunable gains are sent to real IO with `LoggedTunableNumber.ifChanged`, and SPARK MAX hardware runs PID/MAXMotion. `PositionJointIO` separates pivot real, simulation, and replay implementations. Transmission reductions and home angle are explicit in `MechanismConstants`.
