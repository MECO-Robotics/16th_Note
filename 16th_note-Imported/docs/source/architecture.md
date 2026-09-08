# Architecture

## Lifecycle and command ownership

`Robot` extends AdvantageKit's `LoggedRobot`, starts logging, creates
`RobotContainer`, and runs `CommandScheduler` from `robotPeriodic()`.
It schedules the chooser command on autonomous entry, cancels it for teleop,
and cancels commands for test. Mode transitions explicitly stop the tank.

`RobotContainer` assembles `TankDrive` with the IO implementation selected by
`frc.robot.constants.Constants.currentMode`. Its default `runEnd` command
reads the two joysticks and the game-piece vision client, drives only during
teleop, and stops when interrupted. The autonomous chooser registers a
continuous stop command requiring the same subsystem.

## Tank IO boundary

| Layer | Responsibility |
| --- | --- |
| `subsystems/tank/TankDrive` | Input shaping, mixing, voltage limits, disabled gate, logging |
| `subsystems/tank/TankDriveIO` | Sensor/output contract and auto-logged input definition |
| `subsystems/tank/TankDriveIOPWM` | PWM controllers, inversion, MotorSafety |
| `sim/tank/TankDriveIOSim` | Differential-drive physics, simulated wheel state and heading |
| `sim/replay/tank/TankDriveIOReplay` | No-op hardware boundary during log replay |

Positive side voltage means forward motion for that side. Tank inputs use
deadband and signed squaring; arcade input is linear. Voltage requests are
clamped to ±12 V. Disabled or nonfinite voltage requests stop both sides.

## Logging and feedback

`TankDrive.periodic()` updates IO inputs and calls
`Logger.processInputs("TankDrive", inputs)`. Requested left/right voltage is
recorded as `TankDrive/RequestedVolts`. Inputs include applied output voltage,
wheel positions and velocities, heading, and `hasPositionFeedback`.

Real PWM IO reports commanded output multiplied by battery voltage; it is
not an independent measurement of motor terminal voltage. It has no encoders
or gyro and leaves `hasPositionFeedback` false. Simulation supplies model
feedback. Replay restores these inputs from a log without driving hardware.

Real mode writes WPILOG data and publishes NetworkTables; simulation publishes
NetworkTables. Replay reads a source log and writes a `_sim` output log.
Driver joystick and game-piece request decisions do not have a dedicated
logged input layer, limiting reproducibility of those decisions in replay.

## Retained base components

The upstream swerve `Drive`, module IO, odometry threads, PathPlanner utilities,
characterization commands, and reusable mechanism subsystems remain in the
source tree. The active container does not instantiate them or register their
autos. Adding tank pose estimation or closed-loop path following requires
new measured feedback and differential-drive integration; existing swerve
paths cannot simply be enabled in the chooser.
