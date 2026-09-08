# 16th Note — 2026 Tank Robot

This site documents the active `16th_note-Imported/` WPILib project for team
8324. It is based on MECO's `2026_Base_Bot`, with a command-based tank
drivetrain, AdvantageKit logging, and game-piece vision assist.

```{toctree}
:maxdepth: 2
:caption: Project guide

getting-started
architecture
subsystems
customizing
```

## Current capabilities

- Two-stick tank control with PWM Spark Max outputs.
- Trigger-held vision assist with manual override and request freshness checks.
- Separate real, physics simulation, and replay IO implementations.
- Logged drivetrain inputs and requested voltages.
- A stop-only autonomous chooser; no tank odometry or path following yet.

The inherited swerve code, PathPlanner assets, and mechanism libraries remain
available for reuse. They are not instantiated by the active `RobotContainer`.
Start with {doc}`getting-started` for wiring, controls, and build commands.
