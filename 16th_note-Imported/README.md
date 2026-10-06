# 16th Note — 2026 tank robot

The active WPILib project is `16th_note-Imported/`. The sibling `16th_note/`
folder is retained as the original source archive; do not deploy it.

Based on [MECO-Robotics/2026_Base_Bot](https://github.com/MECO-Robotics/2026_Base_Bot)
at `36d35b5eda4d0e2d32c25c65f0b514acf9590ad5`, with the original repository
commits rebased onto that history. The base source and build files live here
to preserve the existing VS Code workspace location.

## Tank operation

- `Robot` uses the base's AdvantageKit logging and command scheduler.
- `RobotContainer` calls `TankDriveIO.fromSparkMax(MotorType.kBrushless)`; the factory
  selects real, simulation, or replay IO using `frc.robot.constants.Constants.currentMode`.
- Wiring: brushless CAN SPARK MAX left ID 1, right ID 2; right output inverted.
  Check wiring and wheel direction with the robot lifted before driving.
- One Xbox or PS4 controller on USB 0 uses left-stick Y for forward/reverse and right-stick X for steering. Forward is negative Y;
  inputs use a 0.02 deadband and signed squaring.
- Hold Xbox RT past halfway or the PS4 R2 button for the existing `GamePieceVision/v1/left`
  assist. Moving either drive axis beyond 0.15 overrides assist. Requests must pass
  the client's enabled, schema, connection and 250 ms freshness checks.
  Assist uses linear mixing capped at 35% output and stops at goal.
- Xbox/PS4 layout is detected automatically on USB 0. Both use left-stick Y
  for forward/reverse and right-stick X for turning. For a manual override, set
  `TankDriveConstants.AUTO_DETECT_CONTROLLER = false` and select the layout with
  `USE_XBOX_CONTROLLER`, then rebuild and deploy. `DriverControls` maps LB/L1 to intake
  and RB/R1 to shoot. Five-NEO SPARK MAX hardware IO, disabled-only referencing,
  Test-mode commissioning and preload autonomous are implemented. REAL settings
  remain unconfigured; see [Fuel commissioning](docs/source/fuel-system.md).
- Commands own the drivetrain; interruption and disable stop both sides.
  Teleop input is ignored in autonomous and test. CAN IO MotorSafety expires
  after 100 ms without feeding.
- Autonomous defaults to a continuous stop; the optional timed backing and preload
  shooting command requires verified fuel/auto settings and a valid intake reference. The inherited swerve PathPlanner
  paths are examples only and are not offered for this tank robot. Real CAN
  IO has no calibrated wheel-position or heading feedback, so pose/path following is not configured.
- Simulation has differential-drive physics with estimated dimensions in
  `TankDriveConstants`; these are not calibrated robot measurements. Replay
  restores logged IO inputs and never allocates motor hardware.

## Controller troubleshooting

Assign the controller to USB 0 in Driver Station and check these dashboard values
while the drivetrain command is running in teleop:

| Key | Expected behavior |
| --- | --- |
| `Drive/ControllerConnected` | `true` with the controller connected |
| `Drive/ControllerLayout` | `Xbox (auto)` or `PS4 (auto)` matching the supplied layout |
| `Drive/ForwardInput` | Changes with left-stick up/down; forward is positive |
| `Drive/TurnInput` | Changes with right-stick left/right; left is positive |

The inputs are published before drivetrain deadband and squaring. They are only
updated during enabled teleop with a connected controller, so displayed input
values can remain from an earlier cycle while disabled or disconnected.

Automatic selection uses Driver Station's Xbox flag or an Xbox/XInput device
name; other named devices use the PS4 layout. Unnamed simulation devices use
`USE_XBOX_CONTROLLER` as a fallback. Only Xbox and PS4 layouts are supported.
If an adapter reports the wrong layout, set `AUTO_DETECT_CONTROLLER = false`
and `USE_XBOX_CONTROLLER = true` for Xbox or `false` for PS4 in
[`TankDriveConstants`](src/main/java/frc/robot/constants/drive/TankDriveConstants.java),
then rebuild and deploy. A PS4 controller presented by an adapter as XInput
should use the Xbox layout.

The Xbox right-stick X axis is 4 and the native PS4 right-stick X axis is 2;
both use axis 1 for left-stick Y. Forcing Xbox mode with native PS4 input can
therefore allow forward/reverse while steering fails. The automatic selection
fix must be deployed to the robot before it changes real driving behavior.

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

For an end-to-end NT4 check of the running simulator, follow
[Live NetworkTables simulation check](docs/source/simulation-check.md).
It drives 18 scenarios and records requested/applied voltage and physics feedback.

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
