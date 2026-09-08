# Subsystems

## Active tank drivetrain

`TankDrive` owns left/right output, input shaping, disabled protection, and
AdvantageKit logging. Its real, simulation, and replay IO implementations
follow the base project pattern. See {doc}`architecture` for the IO contract
and {doc}`getting-started` for controls and wiring.

The active game-piece integration uses `GamePieceVisionClient` to read remote
normalized requests; it is separate from the inherited pose-estimation vision
subsystem described below. `RobotContainer` consumes forward and turn only.

## Available base components

The following reusable libraries are retained from `2026_Base_Bot` but are not
instantiated by the current tank `RobotContainer`.

### Swerve Drive

The inherited `subsystems/drive/Drive` reads gyro/module state, estimates pose,
and converts chassis requests into swerve module setpoints. Its PathPlanner
and characterization support is specific to that drive stack.

### Flywheel

`Flywheel` is the simplest reusable motor subsystem. It is appropriate when one logical mechanism shares a single velocity target.

Use it for:

- Shooter wheels spinning together.
- Conveyor rollers.
- Intake rollers with a shared speed goal.

It supports:

- Velocity control
- Direct voltage control
- Tunable feedforward and feedback gains
- Slew-limited setpoint generation

### PositionJoint

`PositionJoint` covers mechanisms that need bounded positional control, including pivots and elevators.

It supports:

- Position targets with tunable constraints
- Optional dynamic velocity overrides
- Direct voltage control
- Soft range limits
- Gravity-aware feedforward

This is the subsystem you should reach for when the mechanism is fundamentally defined by where it should be, not just how fast it should spin.

### Vision

The vision stack accepts one or more camera implementations and filters observations before passing them into a consumer, usually the drive pose estimator.

Important behavior:

- Rejects invalid or out-of-bounds observations.
- Scales measurement uncertainty from ambiguity, tag count, and target distance.
- Logs accepted and rejected estimates for debugging.

### Sensors and game-piece detection

The repository also includes smaller building blocks for:

- Digital sensors
- Game-piece detection
- Absolute encoders
- PathPlanner utilities

Those classes are intentionally narrow. They are meant to be composed into higher-level robot behaviors rather than expanded into monolithic subsystems.
