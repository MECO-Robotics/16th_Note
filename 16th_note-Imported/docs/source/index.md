# 16th Note — 2026 Tank Robot

This site documents the active `16th_note-Imported/` WPILib project for team
8324. It is based on MECO's `2026_Base_Bot`, with a command-based tank
drivetrain, AdvantageKit logging, and game-piece vision assist.

```{toctree}
:maxdepth: 2
:caption: Project guide

getting-started
simulation-check
fuel-system
architecture
subsystems
customizing
```

## Current capabilities

- Xbox/PS4/Logitech Dual Action arcade control with CAN SPARK MAX outputs.
- Trigger-held vision assist with manual override and request freshness checks.
- Separate real, physics simulation, and replay IO implementations.
- Logged drivetrain inputs and requested voltages.
- Five-NEO SPARK MAX fuel IO, intake referencing and bounded Test-mode commissioning.
- Coordinated intake/shooter/indexer sequence and optional bounded agitation.
- Fuel fault handling and diagnostics; real configuration starts incomplete.
- Default stop-only auto plus optional verified timed backing and preload shooting.
- No tank odometry or path following yet.

The inherited swerve code, PathPlanner assets, and mechanism libraries remain
available for reuse. They are not instantiated by the active `RobotContainer`.
Start with {doc}`getting-started` for wiring, controls, and build commands.
