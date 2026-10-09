# Architecture

## Lifecycle and command ownership

`Robot` extends AdvantageKit's `LoggedRobot`, starts logging, creates
`RobotContainer`, and runs `CommandScheduler` from `robotPeriodic()`.
It schedules the chooser command on autonomous entry, cancels it for teleop,
and cancels commands for test. Mode transitions explicitly stop the tank and fuel mechanism.

`RobotContainer` assembles `TankDrive` through `TankDriveIO.fromSparkMax(MotorType.kBrushless)`.
The factory encapsulates real, simulation, and replay IO selection using
`frc.robot.constants.Constants.currentMode`. It wires the default `TankTeleopCommand`.
The command reads the automatically detected (or manually overridden) Xbox/PS4/Logitech Dual Action
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

The owner confirmed two indexer motors on one shared shaft, with a 1:1 belt drive
from the indexer to the intake rollers and a separate intake-pivot motor. Real IO
commands both shared-shaft motors together for intake collection and shooting
feed, requires both to be healthy, and commissions them only as `INDEXER_PAIR`.
Simulation retains separate intake/feed intent values for game-piece behavior but
models pair commissioning as one linked mechanism.

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

The owner reports a **MAXPlanetary 4:1 gearbox**, a **22T gearbox output sprocket**,
**22T wheel sprockets**, and **3-inch drive wheels**. The resulting motor-to-wheel
reduction is `4 * 22 / 22 = 4:1`. REV specifies an exact 4:1 ratio for the
[MAXPlanetary 4:1 cartridge](https://revrobotics.global/rev-21-2102/).
Nominal travel per motor rotation is `pi * 0.0762 / 4`, approximately
**0.05985 m (2.356 inches)**. Verify effective rolling diameter on the robot.

Simulation now uses this reported reduction and wheel diameter for both sides;
report any left/right differences before hardware distance calibration.
The owner reports **9.414 inches (0.2391156 m)** between the left and right wheel
centers; simulation uses this track width for turning. Mass and rotational
inertia remain estimates. Real encoder-based distance and wheel-only heading
still need implementation and calibration; effective turning track width can
differ from the geometric measurement because of tire scrub and slip.

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

## Planned AprilTag localization and distance-based shooting

The owner plans to use a camera with PhotonVision on an Orange Pi (OPI) for
AprilTag tracking, field localization, and shooter speed selection by distance.
This is planned functionality, not an implemented path in the active tank robot.

The intended data flow is camera/PhotonVision -> timestamped vision pose ->
position/distance estimate combining visible AprilTags and calibrated wheel
encoder movement -> shooter-to-scoring-target distance -> calibrated, tunable RPM targets
for both shooter motors. Use the scoring target's field location rather than
assuming distance to an arbitrary visible AprilTag equals shooting distance.
The distance-to-RPM calibration should be editable and persistent like the intake
angles. Vision acceptance, pose freshness, valid shot range, and behavior when
localization becomes unreliable still need to be defined before automatic feeding.
The owner plans limited use without a gyro; wheel-based turning estimates will
use the reported track width, and drift during turns or slip must be accounted for
when accepting distance estimates. Retain a manual RPM option.

Pending details include the Orange Pi and camera models, PhotonVision version,
camera calibration and robot-relative mounting transform,
drivetrain distance calibration, field/tag layout, and measured shot calibration.
The current game-piece vision assist does not provide this AprilTag localization.
See [PhotonVision pose estimation](https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html).

## Retained base components

The upstream swerve `Drive`, module IO, odometry threads, PathPlanner utilities,
characterization commands, and reusable mechanism subsystems remain in the
source tree. The active container does not instantiate those inherited subsystem wrappers or
register their autos. The fuel IO adapter uses the established REVLib patterns
with its own configuration checks, reference validity and shared stop policy. Adding tank pose estimation or closed-loop path following requires
new measured feedback and differential-drive integration; existing swerve
paths cannot simply be enabled in the chooser.
