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
- One Xbox controller on USB 0 uses left-stick Y for
  forward/reverse and right-stick X for steering. Forward is negative Y; inputs use
  a 0.02 deadband and signed squaring.
- Hold Xbox RT past halfway for the existing `GamePieceVision/v1/left`
  assist. Moving either drive axis beyond 0.15 overrides assist. Requests must pass
  the client's enabled, schema, connection and 250 ms freshness checks.
  Assist uses linear mixing capped at 35% output and stops at goal.
- `DriverControls` uses the Xbox mapping only: left-stick Y drives, right-stick X
  turns, LB runs intake, RB shoots, and RT requests vision assist.
- Commands own the drivetrain; interruption and disable stop both sides.
  Teleop input is ignored in autonomous and test. CAN IO MotorSafety expires
  after 100 ms without feeding.
- Autonomous defaults to a continuous stop; the optional timed backing and preload
  shooting command requires verified fuel/auto settings and a valid intake reference. The inherited swerve PathPlanner
  paths are examples only and are not offered for this tank robot. Real CAN
  IO has no calibrated wheel-position or heading feedback, so pose/path following is not configured.
- Simulation uses the confirmed 4:1 drive reduction, 3-inch wheels, and
  9.414-inch geometric track width. Mass, inertia, effective rolling diameter,
  and effective turning track width still require physical calibration. Replay
  restores logged IO inputs and never allocates motor hardware.

## Hardware map and confirmed ratios

Program these CAN IDs in REV Hardware Client before applying a REAL configuration:

| CAN ID | Mechanism |
| --- | --- |
| 1 | Left drivetrain |
| 2 | Right drivetrain |
| 3 | Primary shooter |
| 4 | Secondary shooter |
| 5 | Primary indexer motor |
| 6 | Secondary indexer motor on the same shaft |
| 7 | Intake pivot |

The two indexer motors always run together. Their shared shaft drives the intake
roller; there is no separate intake-roller motor.

| Mechanism path | Confirmed ratio |
| --- | --- |
| Drive NEO to wheel | 4:1 reduction: 4:1 MAXPlanetary and 22T-to-22T chain stage |
| Each shooter NEO to flywheel | 24T-to-24T, 1:1 |
| Each indexer NEO to shared indexer shaft | 12T-to-24T, 2:1 reduction |
| Shared indexer shaft to intake roller | 26T-to-20T; intake runs 1.3 times indexer-shaft speed |
| Each indexer NEO to intake roller | 20/13, approximately 1.5385 motor rotations per roller rotation |
| Pivot NEO to intake pivot | `(38/22) * (54/12) = 171/22`, approximately 7.7727:1 |

The pivot conversion is approximately 46.3158 intake degrees per motor rotation.
Zero is the upper frame-contact stop, and the currently confirmed full travel is
160 degrees. Shooter targets and feedback use motor RPM; the 1:1 drive makes that
equal to flywheel RPM. The checked-in dashboard starting value is a provisional
1800 RPM and still requires shot testing.

## REAL fuel setup and first startup

Five-NEO fuel hardware IO, disabled-only pivot referencing, Test-mode
commissioning, and preload autonomous are implemented. `FuelConstants.REAL`
deliberately starts unconfigured, so fuel outputs remain blocked until valid
electrical and control settings are applied while disabled.

For the first hardware startup:

Record the measured settings in the
[commissioning values worksheet](COMMISSIONING_VALUES.md) so they can be copied
directly into the permanent REAL configuration.

1. Verify CAN IDs 1–7 and current firmware in REV Hardware Client.
2. Lift the drivetrain and verify left/right direction before floor testing.
3. Enter conservative current limits, voltage caps, test-voltage caps, inversions,
   feedback signs, and the verified shooter coupling under `Fuel/Config/`, then invoke
   **Fuel/Apply configuration (disabled)**.
4. Use Test mode at low output to verify the indexer motors assist each other,
   the intake roller collects in the intended direction, and both shooters spin
   in the shooting direction.
5. Support the intake at its upper stop while disabled and invoke
   **Fuel/Confirm intake at upper stop**. Require `Fuel/Referenced = true` and
   `Fuel/Reference = Upper reference verified`.
6. Verify the 160-degree travel, tune the pivot profile/gains and shooter velocity
   gains, then test intake and shooting with a small fuel load before full capacity.

Dashboard device configuration is temporary and is lost after a robot-program
restart. Copy verified settings into `FuelConstants.REAL`, rebuild, deploy, and
repeat the startup/reference test before relying on the robot at an event. The
accepted feed and agitation angles are the exception: they persist separately
and can be retuned without redeploying.

Autonomous defaults to **Do nothing**. Keep it selected until the REAL fuel
configuration, intake reference, backing voltages, timings, and starting position
have all been tested. See [Fuel commissioning](docs/source/fuel-system.md) for the
full interlocks and handoff procedure. Track remaining event work in the
[Roboticon checklist](ROBOTICON_CHECKLIST.md).

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

## Vision status

The current vision assist consumes the existing `GamePieceVision/v1/left`
NetworkTables feed. Planned PhotonVision AprilTag localization on an Orange Pi,
wheel-assisted field position, and automatic distance-to-shooter-RPM selection
are not implemented yet. The robot currently uses fixed, independently tunable
shooter RPM targets.

## Controller troubleshooting

Assign the controller to USB 0 in Driver Station. Controller identification and
raw-axis diagnostics update in every mode, including **disabled**; checking them
does not require driving the robot.

| Key | Expected behavior |
| --- | --- |
| `Drive/ControllerName` | Device name reported by Driver Station |
| `Drive/ControllerConnected` | `true` with the controller connected |
| `Drive/ControllerLayout` | `Xbox` |
| `Drive/ControllerIsXbox` | Whether Driver Station reports an Xbox mapping |
| `Drive/ControllerPort` | `0` |
| `Drive/ControllerAxisCount` | Number of reported axes; absent raw axes display zero |
| `Drive/RawAxis0` through `Drive/RawAxis5` | Live stick/trigger values, including while disabled |
| `Drive/ForwardInput`, `Drive/TurnInput` | Signed teleop requests before deadband and squaring |

Forward/turn requests update only during enabled teleop with a connected controller
and may retain old displayed values after disabling. Use the raw axes to diagnose
inputs while disabled; raw axes reset to zero on disconnect.

Only the Xbox layout is supported. With the robot disabled, confirm
`Drive/ControllerIsXbox` is true, `Drive/RawAxis1` changes for left-stick Y,
`Drive/RawAxis4` changes for right-stick X, and `Drive/RawAxis3` changes for RT.

## Build and verify

From this directory, using the WPILib 2026 JDK:

```sh
./gradlew build
./gradlew test
./gradlew simulateJava
```

With team 8324 connectivity available, deploy from this directory with:

```sh
./gradlew deploy
```

Enable the simulation GUI explicitly in WPILib when needed; the base disables
it by default for replay compatibility. Team number remains 8324. Wiring and
simulation parameters live in
[`TankDriveConstants`](src/main/java/frc/robot/constants/drive/TankDriveConstants.java);
[vision settings](src/main/java/frc/robot/Constants.java) remain separate.

For an end-to-end NT4 check of the running simulator, follow
[Live NetworkTables simulation check](docs/source/simulation-check.md).
It drives 18 scenarios and records requested/applied voltage and physics feedback.

The test suite covers output limits, forward/reverse/turn behavior, deadband,
disable, command cancellation, vision stop/limits, and simulation movement.
The [robot Wi-Fi helper](scripts/ROBOT-WIFI.md) remains under `scripts/`.

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
See [LICENSE.md](LICENSE.md), [WPILib-License.md](WPILib-License.md), and
[AdvantageKit-License.md](AdvantageKit-License.md).
