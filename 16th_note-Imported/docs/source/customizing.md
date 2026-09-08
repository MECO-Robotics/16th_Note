# Customizing 16th Note

## Configuration locations

| Change | File under `src/main/java/frc/robot/` |
| --- | --- |
| PWM ports, inversion, joystick ports, deadband, simulation estimates | `constants/drive/TankDriveConstants.java` |
| Vision camera and assist limits | `Constants.java` |
| Real/sim/replay mode selection | `constants/Constants.java` |
| Commands, controls, mode-specific IO, autonomous chooser | `RobotContainer.java` |

The two `Constants` classes have different responsibilities. Check the
package before editing or importing one.

## Changing motor hardware

For PWM wiring changes, update `TankDriveConstants`. For CAN motor controllers,
add a `TankDriveIO` implementation and select it in `RobotContainer`'s REAL
branch. Keep positive voltage defined as forward for each side. Keep motor
vendor APIs out of `TankDrive`, and preserve stop behavior and simulation/replay
support. PWM port numbers are not CAN IDs.

## Adding feedback and autonomous

The current robot has no configured real wheel sensors or gyro. Before adding
closed-loop tank autonomous:

1. Implement measured wheel position/velocity and heading in real IO, with
   correct units and signs.
2. Measure drivetrain dimensions and conversion factors.
3. Add differential-drive odometry or pose estimation and its reset behavior.
4. Configure a differential-drive path controller and appropriate robot model.
5. Register commands that require `TankDrive` and stop on completion or interruption.
6. Validate in simulation and on the robot before enabling driving autos.

The inherited swerve `AutoBuilder` setup and example paths are not active tank
configuration. The chooser remains stop-only until that integration is done.

## Adding mechanisms

Follow the base structure: an IO interface, real/sim/replay implementations,
a subsystem that handles behavior and logging, and commands for operator
intent. Existing `Flywheel` and `PositionJoint` libraries can be reused when
they match the hardware. Instantiate only needed mechanisms in `RobotContainer`.

## Validation and documentation

Run `./gradlew build` after code changes. The tank tests cover direction,
deadband, voltage limits, disabled output, cancellation, vision adapter limits
and stop behavior, and simulation movement. They do not validate physical
wiring or the full camera-to-robot integration.

Update the README and these pages when changing wiring, controls, logging,
autonomous support, or setup. See {doc}`getting-started` for the docs build.
