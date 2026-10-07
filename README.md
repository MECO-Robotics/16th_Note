# 16th Note — 2026 REBUILT

Tank-drive robot code with Xbox, PS4, and Logitech Dual Action controls, coordinated intake and shooter
control, preload autonomous, and desktop fuel simulation.

**Open and deploy only [16th_note-Imported](16th_note-Imported/) in WPILib VS Code.**
The sibling `16th_note/` folder is the original project archive.

## Driver controls

Assign the controller to **USB 0** in Driver Station. The code automatically
selects the Xbox, PS4, or Logitech Dual Action layout; switching between them does not require rebuilding.

| Action | Xbox | PS4 | Logitech Dual Action |
| --- | --- | --- | --- |
| Drive forward/backward | Left stick up/down | Left stick up/down | Left stick up/down |
| Turn left/right | Right stick left/right | Right stick left/right | Right stick left/right |
| Lower intake and run rollers once deployed | Hold LB | Hold L1 | Hold button 5 |
| Stop rollers and leave intake down | Release LB | Release L1 | Release button 5 |
| Spin up shooters, then raise intake and feed when ready | Hold RB | Hold R1 | Hold button 6 |
| Stop shooter/indexer requests and return intake down | Release RB | Release R1 | Release button 6 |
| Request existing vision assist | Hold RT past halfway | Hold R2 | Hold button 8 |

Shooting takes priority when both bumpers are held. Release both bumpers after
an enable, reconnection, or interruption before operating the fuel mechanisms.
Manual stick input overrides vision assist. Driving stops on controller disconnect.
Fuel operation requires valid configuration and an intake reference.

If steering does not respond, check `Drive/ControllerLayout` and
`Drive/ControllerConnected` on the dashboard, even while disabled. For the standard Logitech mapping,
`Drive/RawAxis2` should change when moving the right stick sideways. See the
[controller troubleshooting instructions](16th_note-Imported/README.md#controller-troubleshooting)
for axis diagnostics and a manual layout override.

## Build and run

Use WPILib 2026 and its bundled JDK. Run commands from `16th_note-Imported/`.

| Task | macOS / Linux | Windows PowerShell |
| --- | --- | --- |
| Build and test | `./gradlew build` | `.\gradlew.bat build` |
| Desktop simulation | `./gradlew simulateJava` | `.\gradlew.bat simulateJava` |
| Deploy to the robot | `./gradlew deploy` | `.\gradlew.bat deploy` |

Build/deploy also work through WPILib VS Code. Verify the team number in the
active project's WPILib configuration before deploying.

## Hardware and commissioning

| CAN ID | Mechanism |
| --- | --- |
| 1 | Left drivetrain |
| 2 | Right drivetrain |
| 3 | Shooter primary |
| 4 | Intake pivot |
| 5 | Intake rollers |
| 6 | Shooter secondary |
| 7 | Indexer |

All five fuel mechanism motors are NEOs with SPARK MAX controllers. The pivot
uses a 20:1 reduction, with zero at the upper stop and 160 degrees at deployment.
Real mechanism settings still require commissioning; simulation examples do not
configure or validate real outputs. The intake must be referenced each robot-program
session. Autonomous defaults to **Do nothing**; backing and shooting preloads
requires verified settings and a valid reference.

Follow the [fuel commissioning guide](16th_note-Imported/docs/source/fuel-system.md)
for configuration, dashboard commands, Test-mode checks, and the ordered hardware
checklist.

## Simulation and documentation

Desktop simulation includes estimated drivetrain physics, 400 neutral-zone fuel,
pickup, shooting, and a detailed robot model with animated intake. AdvantageScope
visualizes the simulation; the robot program supplies the simulated behavior.
Use the fuel guide's **Prepare pickup practice (disabled)** workflow to start.
Optional trench shooting presets and height limits are simulation-only estimates.

The intake's raised angle is editable in AdvantageScope **Tuning Mode** under
**Tuning → Fuel → FeedDegrees**. Increase it to stop the intake lower. Valid edits
apply while disabled and preserve an existing reference; see the
[angle-tuning instructions](16th_note-Imported/docs/source/fuel-system.md#tune-how-far-the-intake-rises).

- [Active project README](16th_note-Imported/README.md): drivetrain details and development commands.
- [Getting started](16th_note-Imported/docs/source/getting-started.md): setup and controller configuration.
- [Fuel system and simulation](16th_note-Imported/docs/source/fuel-system.md): mechanism operation and practice setup.
- [Detailed AdvantageScope asset](16th_note-Imported/sim/Robot_16thNoteDetailed/README.md): robot model installation.
- [Live simulation checks](16th_note-Imported/docs/source/simulation-check.md): automated simulator exercise.

Software tests cover all three controller layouts and fuel behavior. Motor directions,
mechanical travel, feeding, and shot calibration still need hardware verification.
