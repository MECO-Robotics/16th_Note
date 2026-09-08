# Getting Started

## Open the active project

Install the WPILib 2026 toolchain, including its Java 17 JDK. Clone
`MECO-Robotics/16th_Note` and open **16th_note-Imported/** in WPILib VS Code.
The sibling `16th_note/` directory is the original project archive.
Run all commands below from `16th_note-Imported/`.

```sh
./gradlew build
./gradlew test
```

On Windows, use `./gradlew.bat` in place of `./gradlew`.
The build includes formatting, Java compilation, packaging, and tests.
The team number is 8324 in `.wpilib/wpilib_preferences.json`.

## Wiring and controls

The current configuration preserves the old robot's executable PWM setup.
Actual wiring and inversion still need to be verified on the robot.

| Item | Configuration |
| --- | --- |
| Left Spark Max | PWM 0, not inverted |
| Right Spark Max | PWM 1, inverted |
| Left joystick | USB 0, negative Y drives forward |
| Right joystick | USB 1, negative Y drives forward |
| Vision assist | Hold the right joystick trigger |
| Tank input shaping | 0.02 deadband, signed squaring |
| Manual assist override | Either stick magnitude greater than 0.15 |

Wiring and simulation values live in
`src/main/java/frc/robot/constants/drive/TankDriveConstants.java`.
The old CAN IDs are not used. Check wheel direction with the robot lifted
before floor testing.

## Vision assist

The camera name is `left`, configured in `frc/robot/Constants.java`.
`GamePieceVisionClient` reads `GamePieceVision/v1/left` from NetworkTables.
Assist requires the driver trigger, no manual override, an enabled robot,
a connected camera, schema version 1, an active request, and a frame update
no older than 250 ms. `RobotContainer` additionally restricts driving to teleop.

Forward and turn requests are each scaled by 0.35 and mixed without squaring.
An active at-goal request stops the drivetrain. An invalid or stale request
returns control to the sticks. Strafe is unused by the tank drivetrain.

## Simulation and replay

```sh
./gradlew simulateJava
```

The base disables the simulation GUI by default to support log replay.
Enable the GUI through WPILib's simulation selection when interactive driving
is needed, assign joysticks to slots 0/1, and enable teleop in the simulated
Driver Station. The physics model uses estimated gearing and dimensions;
it is not calibrated to the real robot.

Runtime selection lives in `frc.robot.constants.Constants`, a different class
from the robot-specific vision settings in `frc.robot.Constants`:

- On a roboRIO, `currentMode` always selects `REAL` and PWM hardware IO.
- On desktop, `simMode = Mode.SIM` selects drivetrain physics.
- Set `simMode = Mode.REPLAY` to replay an AdvantageKit log. Keep HAL simulation
  extensions disabled and supply a log when prompted by the replay utility.
  Replay IO allocates no motors; logged inputs are restored by AdvantageKit.

AdvantageKit automatically logs and replays Driver Station state and joystick
inputs. The custom game-piece vision client reads live NetworkTables without
a logged input layer, so vision-assist decisions are not reproduced reliably
from a log alone.

## Deploy and operate

With team 8324 connectivity available, run:

```sh
./gradlew deploy
```

The workstation-specific Wi-Fi ownership helper is documented in
`scripts/ROBOT-WIFI.md`. It can move the USB Wi-Fi adapter between Ubuntu and
the Windows Driver Station VM; it does not deploy or enable the robot.

Autonomous currently offers only **Do nothing**, which continuously stops
both outputs. The inherited swerve paths are not configured for tank use.
Disabled mode and interrupted driving commands stop the drivetrain. Real PWM
outputs also use a 100 ms WPILib MotorSafety expiration.

## Build these docs

From the active project directory:

```sh
python3 -m venv /tmp/16th-note-docs-venv
/tmp/16th-note-docs-venv/bin/python -m pip install -r docs/requirements.txt
/tmp/16th-note-docs-venv/bin/python -m sphinx -W --keep-going -b html docs/source docs/build/html
```

Open `docs/build/html/index.html`. On Windows, create a virtual environment
with `py -m venv` and use its `Scripts/python.exe` executable instead.
