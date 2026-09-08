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

## Project documentation

The Sphinx guide in [docs/source](docs/source/index.md) covers setup, controls,
architecture, and customization. Build it in an isolated Python environment:

```sh
python3 -m venv /tmp/16th-note-docs-venv
/tmp/16th-note-docs-venv/bin/python -m pip install -r docs/requirements.txt
/tmp/16th-note-docs-venv/bin/python -m sphinx -W --keep-going -b html docs/source docs/build/html
```

Open `docs/build/html/index.html`. The GitHub docs workflow builds this site;
a hosted URL depends on this repository's GitHub Pages configuration.

## Attribution

This project retains the reusable source and licenses from MECO's
`2026_Base_Bot`, including Ninjineers and Mechanical Advantage components.
See `LICENSE.md`, `WPILib-License.md`, and `AdvantageKit-License.md`.
