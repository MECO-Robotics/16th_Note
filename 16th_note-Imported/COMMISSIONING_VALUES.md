# 16th Note commissioning values

Replace every `___` below. Use numbers without units, and use only `true` or
`false` for Boolean fields. Leave the confirmed values unchanged. When this is
complete, send this entire file back to Codex so the values can be copied into
`FuelConstants.REAL` and checked before the final deployment.

Repeated agitation is intentionally disabled. Holding Xbox RB raises the intake
to `FeedDegrees`, runs the indexer, and holds that position until RB is released.

## Quick hardware confirmation

| Check | Result |
| --- | --- |
| CAN IDs 1 through 7 visible in REV Hardware Client | `___` |
| Xbox controller works on USB 0 | `___` |
| Left drivetrain moves forward on a forward command | `___` |
| Right drivetrain moves forward on a forward command | `___` |
| Indexer motors 5 and 6 assist each other | `___` |
| Intake roller pulls fuel into the robot | `___` |
| Both shooter wheels move fuel toward the exit | `___` |
| Positive pivot feedback means the intake moves down | `___` |
| Pivot upper-stop reference succeeds | `___` |

```text
Drive/LeftInverted: ___
Drive/RightInverted: ___
```

## Motor configuration

Fill one row after testing that motor at low voltage. `DirectionConfirmed` must
remain `false` until the physical direction has actually been observed.

| Role | CAN ID | CurrentAmps | MaxVolts | TestVoltsCap | Inverted | FeedbackSign (`1` or `-1`) | DirectionConfirmed | kP | kV |
| --- | ---: | ---: | ---: | ---: | --- | ---: | --- | ---: | ---: |
| SHOOTER_PRIMARY | 3 | `___` | `___` | `___` | `___` | `___` | `___` | `___` | `___` |
| SHOOTER_SECONDARY | 4 | `___` | `___` | `___` | `___` | `___` | `___` | `___` | `___` |
| INDEXER_PRIMARY | 5 | `___` | `___` | `___` | `___` | `___` | `___` | `0` | `0` |
| INDEXER_SECONDARY | 6 | `___` | `___` | `___` | `___` | `___` | `___` | `0` | `0` |
| PIVOT | 7 | `___` | `___` | `___` | `___` | `___` | `___` | `0` | `0` |

## Shooter arrangement

If both motors drive the same physical flywheel shaft, select
`COUPLED_FOLLOWER`. If they drive separate flywheel shafts, select `INDEPENDENT`.
Do not guess: trace the shafts and belts on the assembled robot.

```text
Both motors mechanically drive the same shaft: ___
ShooterCoupling (COUPLED_FOLLOWER or INDEPENDENT): ___
FollowerCompatible: ___
FollowerInverted: ___
```

## Pivot control

The confirmed reduction is `171/22`, or approximately `7.7727:1`. The upper stop
is 0 degrees and full deployment is 160 degrees.

```text
Pivot/kP: ___
Pivot/kD: ___
Pivot/GravityVolts: ___
Pivot/MaxVelocity: ___
Pivot/MaxAcceleration: ___
Pivot/MaxDegrees: 160
```

## Intake and shooter sequence

`FeedDegrees` is the raised shooting position. Start shooter testing near 1800
motor RPM, then record the values that work with fuel loaded.

```text
IntakeDegrees: 160
FeedDegrees: ___
AgitationDegrees: COPY FeedDegrees HERE
PositionToleranceDegrees: ___

PrimaryMotorRpm: ___
SecondaryMotorRpm: ___
PrimaryToleranceMotorRpm: ___
SecondaryToleranceMotorRpm: ___
ReadySeconds: ___
SpinupTimeoutSeconds: ___
MotionTimeoutSeconds: ___

IntakeDuty: ___
IndexerDuty: ___

FeedDwellSeconds: 0
AgitationDwellSeconds: 0
AgitationCycles: 0
AgitationEnabled: false
```

## Final loaded tests

| Test | Result |
| --- | --- |
| One fuel intakes and scores | `___` |
| Three fuel intake and score while RB remains held | `___` |
| Maximum carried fuel intake and score while RB remains held | `___` |
| Releasing RB stops the indexer and shooters and lowers the intake | `___` |
| Disabling during intake stops powered motion | `___` |
| Disabling during shooting stops powered motion | `___` |
| Robot restarts, reapplies configuration, references, and scores again | `___` |

## Notes or problems

```text
___
```
