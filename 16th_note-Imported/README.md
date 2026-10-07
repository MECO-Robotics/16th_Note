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
- One Xbox, PS4, or Logitech Dual Action controller on USB 0 uses left-stick Y for forward/reverse and right-stick X for steering. Forward is negative Y;
  inputs use a 0.02 deadband and signed squaring.
- Hold Xbox RT past halfway or the PS4 R2 button for the existing `GamePieceVision/v1/left`
  assist. Moving either drive axis beyond 0.15 overrides assist. Requests must pass
  the client's enabled, schema, connection and 250 ms freshness checks.
  Assist uses linear mixing capped at 35% output and stops at goal.
- Xbox/PS4/Logitech Dual Action layout is detected automatically on USB 0. All use left-stick Y
  for forward/reverse and right-stick X for turning. For a manual override, set
  `TankDriveConstants.AUTO_DETECT_CONTROLLER = false` and select the layout with
  `CONTROLLER_LAYOUT` (`XBOX`, `PS4`, or `LOGITECH_DUAL_ACTION`), then rebuild and deploy. `DriverControls` maps LB/L1 to intake
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

## Tune intake height in AdvantageScope

Connect using **NetworkTables 4 (AdvantageKit)**, disable real hardware,
and turn on Tuning Mode with the slider icon beside the search box. Under
**Tuning → Fuel**, edit **FeedDegrees** and press Enter. Larger angles stop the
intake lower (0 degrees is the upper stop, 160 is deployed). **AgitationDegrees**
sets the intermediate lower agitation endpoint when agitation is enabled.
Valid edits apply live in simulation without re-zeroing an existing reference
or releasing shoot; hardware edits wait until disabled.
Check `AdvantageKit/RealOutputs/Fuel/AngleTuningStatus` for acceptance or rejection.
See [angle tuning](docs/source/fuel-system.md#tune-how-far-the-intake-rises) for
constraints and the full workflow. The last accepted feed/agitation angles save
automatically and reload after restarting, with separate simulation and real-robot
values. Install this code update once; later angle adjustments need no redeploy.

## Controller troubleshooting

Assign the controller to USB 0 in Driver Station. Controller identification and
raw-axis diagnostics update in every mode, including **disabled**; checking them
does not require driving the robot.

| Key | Expected behavior |
| --- | --- |
| `Drive/ControllerName` | Device name reported by Driver Station |
| `Drive/ControllerConnected` | `true` with the controller connected |
| `Drive/ControllerLayout` | `Xbox (auto)`, `PS4 (auto)`, or `Logitech Dual Action (auto)` |
| `Drive/ControllerIsXbox` | Whether Driver Station reports an Xbox mapping |
| `Drive/ControllerPort` | `0` |
| `Drive/ControllerAxisCount` | Number of reported axes; absent raw axes display zero |
| `Drive/RawAxis0` through `Drive/RawAxis5` | Live stick/trigger values, including while disabled |
| `Drive/ForwardInput`, `Drive/TurnInput` | Signed teleop requests before deadband and squaring |

Forward/turn requests update only during enabled teleop with a connected controller
and may retain old displayed values after disabling. Use the raw axes to diagnose
inputs while disabled; raw axes reset to zero on disconnect.

Automatic selection uses Driver Station's Xbox flag or an Xbox/XInput device name
first, then recognizes `Dual Action`/`DualAction` as Logitech. Other named devices
retain the PS4 fallback. Unnamed simulation devices use `CONTROLLER_LAYOUT`.
Only Xbox, PS4, and Logitech Dual Action layouts are supported.

To force a layout, set `AUTO_DETECT_CONTROLLER = false` and `CONTROLLER_LAYOUT`
to `Layout.XBOX`, `Layout.PS4`, or `Layout.LOGITECH_DUAL_ACTION` in
[`TankDriveConstants`](src/main/java/frc/robot/constants/drive/TankDriveConstants.java),
then rebuild and deploy. The enum replaces the former `USE_XBOX_CONTROLLER` flag.
A gamepad exposed through an XInput wrapper should use the Xbox layout.

For a standard Logitech Dual Action, left-stick Y is axis 1, right-stick X is
axis 2, intake is button 5, shooting is button 6, and vision assist is button 8.
These are the same stick/shoulder indices used by the existing PS4 controls;
adding a Logitech name does not by itself prove that a steering issue is fixed.
With the robot disabled, confirm `Drive/RawAxis2` changes when moving the right
stick sideways. If it does not, record which axis changes and the reported device
name/layout before changing mappings. Xbox right-stick X uses axis 4 instead.
The standard Windows mapping is also recorded in the
[SDL controller database](https://github.com/mdqinc/SDL_GameControllerDB/blob/master/gamecontrollerdb.txt)
(SDL button numbers are zero-based; WPILib button numbers are one-based).

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
