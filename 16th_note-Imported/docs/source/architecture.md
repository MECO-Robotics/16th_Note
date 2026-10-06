# Architecture

## Lifecycle and command ownership

`Robot` extends AdvantageKit's `LoggedRobot`, starts logging, creates
`RobotContainer`, and runs `CommandScheduler` from `robotPeriodic()`.
It schedules the chooser command on autonomous entry, cancels it for teleop,
and cancels commands for test. Mode transitions explicitly stop the tank and fuel mechanism.

`RobotContainer` assembles `TankDrive` through `TankDriveIO.fromSparkMax(MotorType.kBrushless)`.
The factory encapsulates real, simulation, and replay IO selection using
`frc.robot.constants.Constants.currentMode`. It wires the default `TankTeleopCommand`.
The command reads the automatically detected (or manually overridden) Xbox/PS4
layout through `DriverControls` and the game-piece vision client. It drives only
during enabled teleop with a connected controller and stops when interrupted. The autonomous chooser registers a
continuous stop command requiring the same subsystem.

## Fuel coordination

`RobotContainer` also creates `FuelSystem` with its own default command and
subsystem requirement. `FuelSequencer` coordinates the pivot, rollers and shooter
without requiring the drivetrain in teleop. Test commissioning and preload auto
require both fuel and drivetrain. SIM selects `FuelIOSim`; REAL selects
`FuelIOSparkMax`, which only allocates devices with valid electrical configuration;
REPLAY uses no-op hardware IO. `FuelConfiguration` separates real settings from
simulation examples. Explicit disabled dashboard actions apply snapshots and
reference the upper stop. The fuel
subsystem logs feedback, operator requests and sequence time together. See
{doc}`fuel-system` for the operating sequence and remaining hardware integration.

## Tank IO boundary

| Layer | Responsibility |
| --- | --- |
| `subsystems/tank/TankDrive` | Input shaping, mixing, voltage limits, disabled gate, logging |
| `subsystems/tank/TankDriveIO` | Sensor/output contract, auto-logged inputs, and mode-aware SPARK MAX IO factory |
| `subsystems/tank/TankDriveIOSparkMax` | CAN SPARK MAX controllers, inversion, MotorSafety |
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

Real CAN IO reports SPARK MAX applied output multiplied by bus voltage.
Wheel distance and heading are not calibrated, so `hasPositionFeedback` remains false. Simulation supplies model
feedback. Replay restores these inputs from a log without driving hardware.

Real mode writes WPILOG data and publishes NetworkTables; simulation publishes
NetworkTables. Replay reads a source log and writes a `_sim` output log.
AdvantageKit automatically logs and replays Driver Station state and joystick
inputs. The custom game-piece client reads live NetworkTables without a logged
input layer, limiting reproducibility of vision-assist decisions in replay.

## Retained base components

The upstream swerve `Drive`, module IO, odometry threads, PathPlanner utilities,
characterization commands, and reusable mechanism subsystems remain in the
source tree. The active container does not instantiate those inherited subsystem wrappers or
register their autos. The fuel IO adapter uses the established REVLib patterns
with its own configuration checks, reference validity and shared stop policy. Adding tank pose estimation or closed-loop path following requires
new measured feedback and differential-drive integration; existing swerve
paths cannot simply be enabled in the chooser.
