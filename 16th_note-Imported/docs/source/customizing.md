# Customizing

Edit `constants/drive/TankDriveConstants.java` for wiring, inversion, controller port, deadband, and simulation estimates. Mode selection is in `constants/Constants.java`.

Declare subsystems and commands in `RobotContainer`. Follow the rebuilt subsystem/IO pattern with real, simulation, and replay implementations. Keep vendor calls in IO.

Mechanism CAN IDs and motor-to-mechanism reductions live in `constants/MechanismConstants.java`. Hardware records and default gains are in `constants/subsystems/IntakeConstants.java` and `ShooterConstants.java`. Shooter/pivot PID gains and presets are tunable through `/TunableNumbers`. Shooter units are mechanism rotations/second; pivot units are mechanism rotations. Driving autonomous requires measured wheel feedback, heading, geometry, and odometry.
