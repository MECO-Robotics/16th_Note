# Live NetworkTables simulation check

The robot supports the Xbox controller mapping only. Simulation harnesses send
Xbox axes and buttons: left Y on axis 1, RT on axis 3, right X on axis 4, LB on
button 5, and RB on button 6.

This check runs the actual robot program and uses a separate NT4 client to
observe drivetrain outputs and simulated feedback. A HAL WebSocket connection
supplies simulated Driver Station modes and joystick packets; camera requests
are published through NetworkTables. No robot hardware is involved.

## Run it

Use the WPILib 2026 JDK and run these commands from `16th_note-Imported/`.
Keep `frc.robot.constants.Constants.simMode` set to `Mode.SIM`.

Prepare the client once:

```sh
python3 -m venv /tmp/16th-note-sim-check
/tmp/16th-note-sim-check/bin/pip install -r scripts/sim-check-requirements.txt
```

In one terminal, start the simulator with the optional WebSocket extension:

```sh
./gradlew simulateJava -PntCheck
```

Wait for `Robot program startup complete`. In a second terminal:

```sh
/tmp/16th-note-sim-check/bin/python scripts/check-sim-networktables.py
```

To run the longer pickup-and-score exercise instead, restart or leave the
disabled simulator running and use:

```sh
/tmp/16th-note-sim-check/bin/python scripts/check-sim-match-route.py
```

That checker invokes **Fuel/Sim/Prepare full-field route (disabled)**, then
drives with Xbox joystick packets. It collects six staged fuel at each of the
approximate blue depot, blue outpost, and neutral-zone locations, returns to the
close blue-hub shot after each pickup, and requires all 18 fuel to score. Its
upper-field route avoids the bump. Every sampled robot pose is checked against
both blue bump footprints expanded by the conservative 0.32 m robot radius.

The checker connects only to localhost: NT4 on port 5810 and HAL WebSockets
on port 3300. Do not connect another Driver Station or simulator controller
during this run. The checker controls modes and joysticks and disables the
simulated robot in its cleanup path. Stop the simulator with Ctrl+C afterward.
Stop it before rebuilding so native libraries are not replaced while loaded.
The `ntCheck` property is optional and does not enable WebSockets in the
normal build/simulation configuration. Do not enable this extension for replay.

## Observed topics

| NetworkTables path | Meaning |
| --- | --- |
| `/AdvantageKit/RealOutputs/TankDrive/RequestedVolts` | Left/right voltage requests; `RealOutputs` is AdvantageKit's normal non-replay output namespace, including simulation |
| `/AdvantageKit/TankDrive/LeftAppliedVolts` | Left simulated applied voltage |
| `/AdvantageKit/TankDrive/RightAppliedVolts` | Right simulated applied voltage |
| `/AdvantageKit/TankDrive/LeftPositionMeters` | Left simulated distance |
| `/AdvantageKit/TankDrive/RightPositionMeters` | Right simulated distance |
| `/AdvantageKit/TankDrive/LeftVelocityMetersPerSecond` | Left simulated velocity |
| `/AdvantageKit/TankDrive/RightVelocityMetersPerSecond` | Right simulated velocity |
| `/AdvantageKit/TankDrive/HeadingRadians` | Simulated heading, positive counterclockwise |
| `/AdvantageKit/TankDrive/HasPositionFeedback` | True for physics simulation |
| `/AdvantageKit/DriverStation/Enabled` | Logged enabled state |
| `/AdvantageKit/DriverStation/Joystick0/AxisValues` | Logged Xbox axes (left Y: 1; RT: 3; right X: 4) |
| `/SmartDashboard/Auto Choices/options` | Stop-only autonomous option |
| `/SmartDashboard/Fuel/Sim/PoseX`, `PoseY`, `HeadingDegrees` | Live route-check pose |
| `/SmartDashboard/Fuel/Sim/HeldCount`, `LaunchedCount`, `ScoredCount` | Live route-check fuel totals |

The checker publishes camera state below `/GamePieceVision/v1/left` and
normalized requests below its `driveRequest` subtable. Each fresh frame
increments `frameSequence`; the stale-request check deliberately stops updates.

## Checks and expected results

The 18 phases verify requested and applied voltages, enabled state, finite
physics feedback, and the sign of wheel velocity when driving.

| Scenario | Expected left/right volts |
| --- | --- |
| Full forward | +12 / +12 |
| Full reverse | -12 / -12 |
| Tank turn | -12 / +12 |
| Vision forward request 1.0 | +4.2 / +4.2 |
| Vision turn request +0.5 | -2.1 / +2.1, increasing heading |
| Full-stick manual override | +12 / +12 |
| Deadband, trigger released, at goal | 0 / 0 |
| Stale frame, wrong schema, disconnected camera | 0 / 0 |
| Autonomous and test, even with stick input | 0 / 0 |
| Return to teleop with full sticks | +12 / +12 |
| Disabled, including after motion | 0 / 0 |

Results are written to `build/sim-networktables-results.json`. A failed
assertion produces a nonzero exit status. The zero-output checks allow the
physics model to coast; they do not assert instant zero velocity.

The full-field route writes `build/sim-match-route-results.json`, including the
pickup pose for each source and every sampled route pose. The source positions,
bump rectangles, ball flight, and hub opening are practice approximations. The
route check proves command integration and simulated geometry; it does not
claim official field-physics accuracy.

This verifies the running Java robot, HAL mode/input handling, NetworkTables
transport, vision request gates, output mixing, and simulated physics. It does
not validate real motor wiring, calibrated drivetrain parameters, camera
image processing, or replay of the custom vision inputs.
