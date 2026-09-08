# 16th Note — 2026 tank robot

The active WPILib project is `16th_note-Imported/`. The sibling `16th_note/`
folder is retained as the original source archive; do not deploy it.

Based on [MECO-Robotics/2026_Base_Bot](https://github.com/MECO-Robotics/2026_Base_Bot)
at `36d35b5eda4d0e2d32c25c65f0b514acf9590ad5`, with the original repository
commits rebased onto that history. The base source and build files live here
to preserve the existing VS Code workspace location.

## Tank operation

- `Robot` uses the base's AdvantageKit logging and command scheduler.
- `RobotContainer` selects `TankDriveIOPWM`, `TankDriveIOSim`, or
  `TankDriveIOReplay` using `frc.robot.constants.Constants.currentMode`.
- Wiring: PWM Spark Max left port 0, right port 1; right output inverted.
  These preserve the old executable code, not the unused CAN constants.
  Check wiring and wheel direction with the robot lifted before driving.
- Two joysticks on USB 0/1 control the left/right sides. Forward is negative Y;
  inputs use a 0.02 deadband and signed squaring.
- Hold the right joystick trigger for the existing `GamePieceVision/v1/left`
  assist. Moving either stick beyond 0.15 overrides assist. Requests must pass
  the client's enabled, schema, connection and 250 ms freshness checks.
  Assist uses linear mixing capped at 35% output and stops at goal.
- Commands own the drivetrain; interruption and disable stop both sides.
  Teleop input is ignored in autonomous and test. PWM MotorSafety expires
  after 100 ms without feeding.
- Autonomous defaults to a continuous stop. The inherited swerve PathPlanner
  paths are examples only and are not offered for this tank robot. Real PWM
  IO has no encoders or gyro, so pose/path following is not configured.
- Simulation has differential-drive physics with estimated dimensions in
  `TankDriveConstants`; these are not calibrated robot measurements. Replay
  restores logged IO inputs and never allocates motor hardware.

## Build and verify

From this directory, using the WPILib 2026 JDK:

```sh
./gradlew build
./gradlew test
./gradlew simulateJava
```

Enable the simulation GUI explicitly in WPILib when needed; the base disables
it by default for replay compatibility. Team number remains 8324. Wiring and
simulation parameters live in `constants/drive/TankDriveConstants.java`;
vision settings remain in `frc/robot/Constants.java`.

The test suite covers output limits, forward/reverse/turn behavior, deadband,
disable, command cancellation, vision stop/limits, and simulation movement.
The robot Wi-Fi helper remains under `scripts/`.

## Upstream template reference

The following describes reusable base components. Its swerve setup is not the
active tank configuration above.

# Ninjineers 2025 Base Robot Code

FRC Robot Code has gotten significantly more advanced in recent years with brushless motors, external CAN sensors, vision, and more complicated odometry. Ninjineers has realized that these advancements are not possible over the course of a single season so we created this base code to allow teams to focus on the emergent behavior of their robot instead of getting bogged down in the weeds of motor control. In developing this template we tried to create sensible defaults for TalonFX's and Spark Max's for motor control, and integrated Mechanical Advantage's Drivetrain and Vision projects (with modification to enable more modular configuration). These defaults work for us but you are free to create your own [IO layers](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces) or bespoke Subsystems. We would love it if you create a pull request to share your code with other teams.

## Getting Started

1. Create a fork of this repo, this will allow you to pull the latest changes in this repo directly from the Github UI.
2. Pull your fork
3. Split up your robot into different subsystems (Check out the [Subsystems](#subsystems) section)

## Documentation

This repository now includes a Sphinx documentation site under `docs/` using the
[Furo theme](https://github.com/pradyunsg/furo).

Hosted docs: https://meco-robotics.github.io/2026_Base_Bot/

Build it locally with:

```powershell
cd docs
py -m pip install -r requirements.txt
py -m sphinx -b html source build/html
```


## Subsystems

We split our robot into many small subsystems to allow for heavy reuse of subsystems (this is why we have only two motor control subsystems).

* Flywheel

  Flywheel is the more basic of the two subsystems and allows for controlling the velocity or voltage of a single motor or a group of motors. The biggest constraint with a Flywheel subsystem is it can only have one velocity setpoint or velocity measurement. This means that every motor will be spinning together (probably in the same gearbox).

* PositionJoint

  PositionJoints are more complicated as they can represent a Pivot or Elevator. A PositionJoint supports Position and Voltage control modes. Just like the Flywheel a single PositionJoint can also only have one position setpoint and position output but can have multiple motors.

### Splitting a Subsystem

Many of what we want to think of as a single robot Subsystems will need to be represented as multiple Subsystems in code. For example: a shooter from Crescendo might have a set of left wheels and a set of right wheels that spin at different velocities to create spin. This should be represented as two Flywheels (one for the right and one for left). The differential velocity is a more complicated emergent behavior that should be controlled at the command level not the subsystem level.

## Flywheel Constants

Flywheels have mechanism gains and motor configs:

* Motor Configs:

  * canIds: List of canIDs for motors in group. First canID will be master

  * reversed: Reverse config for each motor: first boolean will reverse the master motor, the next booleans will reverse the follower motors relative to the master motor

  * gearRatio: ratio of motor revolutions (rotations) to mechanism revolutions (rotations or radians)

  * canBus: For TalonFX can be name of CANivore or "rio", unused on SparkMax

* Gains:

  Feedback:

  * kP: (volts / rotation / second)

  * kI: (volts / rotation)

  * kD: (volts / rotation / second^2)

  Feedforward:

  * kS: (volts)

  * kV: (volts / rotation / second)

  * kA: (volts / rotation / second^2) (Not recommended to be non 0) (Not used in SparkMax control)

  Tolerance:

  * kTolerance: Velocity tolerance for mechanism to be considered at setpoint

## PositionJoint Constants

* HardwareConfig
  * canIds: List of canIDs for motors in group. First canID will be master

  * reversed: Reverse config for each motor: first boolean will reverse entire group, next booleans are all relative to master motor

  * gearRatio: ratio of motor revolutions (rotations) to mechanism units, e.g. rotations, radians, or meters.

  * currentLimit: current limit of the motor (Amps)

  * gravity: GravityType.CONSTANT for a mechanism where gravity acts in a constant way (e.g. turret or elevator), GravityType.COSINE for pivots with a 0 position horizontal, and GravityType.SINE for pivots with a 0 position vertical (not supported on TalonFX)

  * encoderType: Use EncoderType.INTERNAL to use the motor's internal encoder for relative positioning, EncoderType.EXTERNAL_CANCODER to use a CANCoder for absolute positioning Encoder_Type.EXTERNAL_DIO to use an external encoder connected to the Rio's DIO ports for absolute positioning, and Encoder_Type.EXTERNAL_SPARK to use an external encoder connected to the SPARK MAX motor controller for absolute positioning (not supported for TalonFX).

  * encoderID: The CAN/DIO ID for the external encoder, can be left as 0 or -1 if using EncoderType.INTERNAL or EncoderType.EXTERNAL_SPARK.

  * encoderOffset: The offset of the absolute encoder, in rotations. If using EncoderType.INTERNAL or EncoderType.EXTERNAL_SPARK an empty Rotation (new Rotation2d()) can be used.

  * canBus: For TalonFX can be name of CANivore or "rio", unused on SparkMax

* Gains

  Feedback:
  * kP: (volts / rotation / second)
  * kI: (volts / rotation)
  * kD: (volts / rotation / second^2)

  Feedforward:
  * kS: (volts)
  * kV: (volts / rotation / second)
  * kA: (volts / rotation / second^2) (Not recommended to be non 0) (Not used in SparkMax control)
  * kG: (volts) Feedforward to compensate for effects of gravity

  Trapezoidal:
  * kMaxVelo: (rotation / second)
  * kMaxAccel: (rotation / second^2)

  Positional Bounds:
  * kMinPosition: (rotation)
  * kMaxPosition: (rotation)

  Tolerance:
  * kTolerance: Position tolerance for joint to be considered at setpoint

  Default Setpoint:
  * kDefaultSetpoint: (rotation) the setpoint the mechanism should go to upon intialization
