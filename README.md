# 16th Note — tank robot

Team 9924. Uses the rebuilt command/subsystem/IO structure and AdvantageKit logging.

## Match logs

At startup, the robot records AdvantageKit `.wpilog` files to writable USB storage at `/U/logs`, or falls back to `/home/lvuser/logs` onboard when USB is missing or unwritable. Logs include Driver Station match information, controller inputs, and mechanism telemetry. AdvantageScope can open these files. The selected directory is published at `/AdvantageKit/RealOutputs/Logging/Directory` and recorded in log metadata as `LogDirectory`.

Storage selection happens at program startup; restart the robot program after inserting or replacing USB storage. Monitor onboard free space and download logs regularly. This does not switch destinations during a run if USB is unplugged or fills up.

Xbox USB 0 controls:

| Control | Action |
| --- | --- |
| Left Y / right X | Arcade drive / turn |
| RT / LT | Intake forward / reverse, proportional up to 6 V |
| RB / LB | Indexer forward / reverse at 6 V |
| A / X | Shooter PID at 50% / 75% geared free speed (about 2401 / 3602 wheel RPM) |
| B | Shooter 0 V |
| POV Down / Up | Pivot stow +0.25 rotations (+90 degrees) / deploy -1/6 rotations (-60 degrees) |
| POV Right | Pivot +1/24 rotations (+15 degrees), tunable intermediate preset |

Shooter and pivot presets remain active after button release. Roller controls stop on release; opposing controls cancel. Disable stops all mechanisms.

Autonomous routines are loaded from `src/main/deploy/pathplanner/autos` through PathPlanner's `AutoBuilder` chooser. The tank drivetrain follows PathPlanner paths with its integrated NEO encoders and calculated differential-drive heading. Available named commands are `DeployIntake`, `RaiseIntakeToFeed`, `StowIntake`, `SpinIntake`, `StopIntake`, `SpinUpShooter`, `AutoSpinUp`, `StopShooter`, `FeedIndexer`, `FeedRollers`, `StopIndexer`, `IdleRollers`, `Agitate`, `Shoot`, `Shoot75Percent`, `ShootFullSpeed`, and `StopAll`; enter these exact names in PathPlanner named-command blocks. Driving and turning belong in PathPlanner path blocks. **Back Up and Shoot** resets odometry, follows **Back up and Shoot**, then runs `Shoot75Percent`. **Left trench shoot** and **Right trench shoot** start at 0 degrees with the intake facing the field, follow their matching mirrored paths, then run `ShootFullSpeed`.

`SpinIntake`, `SpinUpShooter`, and `FeedIndexer` keep running until interrupted, so use them inside a PathPlanner parallel deadline/race group or follow them with their matching stop command. `DeployIntake`, `RaiseIntakeToFeed`, `StowIntake`, and `Shoot` finish on their own.

`Shoot75Percent` and `ShootFullSpeed` hold the pivot at the agitation preset and command `/TunableNumbers/Auto/Shoot/BackupSpeed_Rps` and `/TunableNumbers/Auto/Shoot/FullSpeed_Rps`, which default to 75% and 100% geared free speed. They wait for the measured flywheel speed to enter the configured shooter tolerance before feeding the indexer. If `/TunableNumbers/Auto/Shoot/ReadyTimeoutSeconds` expires first, they report an error and do not feed. Feed duration and voltage remain tunable at `/TunableNumbers/Auto/Shoot/{FeedSeconds,IndexerVolts}`.

Path following uses `/TunableNumbers/Drive/Auto/{kS,kV,kA,kP}`. The initial values are model estimates and must be checked with the robot safely raised before running on the field. PathPlanner robot mass, moment of inertia, footprint, and maximum speed are also preliminary; the known 3-inch wheel diameter, 4:1 reduction, and 9.414-inch track width are configured.

SPARK MAX wiring: drive CAN 1/2, indexer 3, pivot 5, intake 6, shooter 7. CAN 4 is unused; vision is disabled.

Gear ratios in `MechanismConstants` use motor rotations per mechanism rotation: shooter 26:22, pivot 171:22. Drive reduction is 4:1 in `TankDriveConstants`. Separate intake/indexer reductions remain unmeasured; their telemetry uses motor rotations/RPM.

Tune P/I/D and feedforward under `/TunableNumbers/Shooter/Gains` and `/TunableNumbers/Pivot/Gains`. Both use the rebuilt project's `LoggedTunableNumber` and `ifChanged` gain-update pattern. Real closed-loop control uses SPARK MAX onboard PID/MAXMotion; gains are not the former software voltage PID gains. Shooter control/telemetry and gains use rotations/second (RPS), matching the rebuilt IO. Shooter A/X presets are `/TunableNumbers/Shooter/Presets/A_Rps` and `X_Rps`. Pivot P starts at `10/12` in controller output units per rotation. These are preliminary settings, not hardware tuning. Pivot output is capped at 25% duty (3 V at 12 V).

Pivot angle zero is horizontal. `/TunableNumbers/Pivot/Gains/kG` is the cosine gravity compensation voltage: zero contribution at +90 degrees, maximum at 0 degrees. Positive compensation commands increasing angle toward the stowed position. The pivot motor inversion is false (reversed from its previous true configuration). `kG` starts at 0 until hardware tuning. `HorizontalRotations` is no longer used. The pivot simulation does not model physical gravity.

At robot-program startup the relative encoder is seeded to home (+90 degrees). This assumes the intake is physically at home whenever the program starts; it cannot detect movement while unpowered. Disable stops outputs rather than driving home. Reference status is diagnostic only: a controller reset does not latch a motion block. If the relative encoder loses its physical angle, place the intake at home while disabled and use the optional **Pivot/Reference home (disabled)** command. Disabled/disconnected and invalid-gain protection still apply.

Pivot presets are `/TunableNumbers/Pivot/Presets/{StowRotations,IntermediateRotations,DeployedRotations}`. All positions are absolute mechanism rotations; allowed travel is 150 degrees, from -60 to +90 degrees (-1/6 to +1/4 rotations).

Rebuilt alignment: `FlywheelIO.fromSparkMax(name, hardware)` and the configured `PositionJointIO` factory select real/simulation/replay; `RobotContainer` wires hardware records from `constants/subsystems`. Presets use the base `holdVelocity`/`holdPosition` command pattern. The implementations are single-motor adaptations: tank drive, startup home seeding, conservative output caps, and additional disable/reference checks are specific to this robot. The desktop plant is an estimate, not a full copy of the rebuilt simulation.

Simulation checks: `scripts/check-sim-networktables.py` and `scripts/check-sim-mechanisms.py`, with the simulator started using `-PntCheck`.

Use WPILib 2026 and Java 17. Run `./gradlew.bat build`, `./gradlew.bat simulateJava`, or `./gradlew.bat deploy`.
