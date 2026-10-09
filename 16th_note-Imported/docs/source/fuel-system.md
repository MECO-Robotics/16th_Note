# Fuel hardware, commissioning and autonomous

## Current implementation and release state

The active `16th_note-Imported` project implements real SPARK MAX IO, five-motor
fuel coordination, disabled-only intake referencing, deadman commissioning,
software simulation, and **Back up and shoot preloads** autonomous.

**The shipped REAL configuration is deliberately unconfigured.**
`FuelConstants.REAL` uses `FuelConfiguration.unconfigured()`. No fuel motor is
allocated until its electrical configuration is valid. Applying temporary
configuration while disabled can make individual mechanisms available for
commissioning without enabling the complete automatic sequence. SIM uses its
own example configuration; those numbers are not robot measurements.

The drivetrain retains CAN IDs 1–2 and its existing controls. Every mechanism
motor is a brushless NEO controlled by a SPARK MAX.

### Consecutive CAN assignments

The owner requested consecutive IDs to make the motor assignments easy to track.
Keep the drivetrain at 1–2 and group the paired shooter and indexer motors:

| CAN ID | Software role | Physical motor label |
| --- | --- | --- |
| 1 | `LEFT_CAN_ID` | Left drivetrain |
| 2 | `RIGHT_CAN_ID` | Right drivetrain |
| 3 | `SHOOTER_PRIMARY` | Shooter A — 14T motor pulley driving 22T flywheel pulley |
| 4 | `SHOOTER_SECONDARY` | Shooter B — matching shooter drive |
| 5 | `INDEXER_PRIMARY` | Indexer A — first motor on the shared indexer shaft |
| 6 | `INDEXER_SECONDARY` | Indexer B — second motor on the same indexer shaft |
| 7 | `PIVOT` | Intake pivot — chain drive that raises/lowers the intake |

There is no separate intake-roller motor: the shared indexer shaft
drives the intake rollers through the reported 1:1 belt. Label the physical motors
A/B to match this table; the labels do not imply left/right mounting or direction.

The software mapping now matches this table. Program these IDs into the physical
SPARK MAX controllers before applying a real configuration; robot code cannot
change device IDs. Intake collection and shooting feed both command indexer motors
5 and 6 together. Their per-controller inversion fields must be set so both motors
assist each other on the shared shaft. If either motor is unhealthy, automatic
fuel operation and `INDEXER_PAIR` commissioning are blocked. Shooter
shaft coupling and directions still require confirmation. The owner reports a
14T motor pulley driving a 22T flywheel pulley on both sides, superseding the
earlier CAD-inferred 1:1 estimate.

## Reported shooter geometry

The following dimensions were supplied by the robot owner:

| Measurement | Value |
| --- | --- |
| Shooter wheel diameter | 4 inches (101.6 mm) |
| Ball compression against the rear shooter plate | 1/4 inch (6.35 mm), as reported |
| Intake compression | Not yet measured |
| Shooter drive arrangement | Two motors and two belts; matching drive arrangement on both sides, as reported |
| Primary shooter pulley sizes | 14T motor driving 22T flywheel, confirmed by owner on 2026-10-08 |
| Secondary shooter pulley sizes | 14T motor driving 22T flywheel, confirmed by owner on 2026-10-08 |
| Motor rotations per flywheel rotation | 22/14, approximately 1.5714:1, on both sides |
| Belt specification | 60 teeth, 5 mm pitch: 300 mm pitch length, as reported |

The previous 24T/24T inference assumed equal pulleys from block CAD; it is no
longer the working ratio. The screenshot's 3.543-inch shaft spacing (3.453 inches
in the written message), the reported 0.725-inch curve radii, and the belt
specification still need reconciliation with the actual 14T/22T drive geometry.

With the confirmed pulley order,
`flywheel RPM = motor RPM * 14 / 22`, and
`motor RPM = flywheel RPM * 22 / 14`. For example, 3000 motor RPM corresponds
to approximately 1909 flywheel RPM. Two motors do not double the
ratio. This information does not establish whether the motors drive a shared
shaft or separate shafts, so it does not select follower mode or confirm coupling.

The compression measurement describes ball squeeze, not the absolute wheel-to-plate
gap. Confirm the measurement reference when documenting that gap.

Wheel diameter matters for surface speed: `surface speed = pi * diameter * wheel RPM / 60`.
For these wheels, use 0.1016 m for diameter to obtain m/s. Motor RPM must first be
converted to wheel RPM using the actual shooter ratio. Ball exit speed still
requires measurement; wheel slip, compression, and contact geometry affect how
the ball accelerates. These dimensions alone do not establish a calibrated RPM
target or justify changing the simulation's launch-speed calibration.

## Reported intake and indexer belt measurements

On 2026-10-08, the owner supplied these pulley radii. The largest pulley moves
the whole intake (pivot), rather than spinning an intake or indexer roller.
The table is a preliminary interpretation of the paths; motor identities,
intermediate shaft connections, and whether these are pitch radii remain unconfirmed.

The owner subsequently clarified that there are **three separate motors**:
one drives a **chain that pivots the intake**, and the other two both drive
**the same indexer shaft**, now confirmed by the owner. The owner
reports a **1:1 belt drive from the indexer to the intake rollers**, superseding
the earlier 1.300 speed estimate for that connection. The two indexer motors need
coordinated output and verified directions so they assist each other on the shared
shaft. The intake rollers and indexer are mechanically linked and cannot be
commanded as independently stoppable mechanisms under this reported arrangement.
The CAN assignments are now established above, and the software commands the pair
together. Equal motor-to-shaft ratios, motor directions, and any pivot gearbox
reduction still need physical confirmation before applying the real configuration.

| Interpreted belt path | Driving radius | Driven radius | Output speed / input speed |
| --- | --- | --- | --- |
| Indexer-side shaft to intake roller (earlier measurements) | 0.815 in | 0.627 in | Superseded by owner's reported 1:1 ratio; measured surfaces/path need reconciliation |
| Intake pivot chain stage (earlier radius measurements) | 0.376 in | 1.593 in | Approximately 0.236 if pitch radii (4.237:1 reduction); tooth counts needed |
| Motor-side pulley to indexer | 0.376 in | 0.815 in | Approximately 0.461 (2.168:1 reduction) |
| Second indexer motor to the shared indexer shaft | 0.376 in | 0.815 in | Approximately 0.461 (2.168:1 reduction); pitch radii/tooth counts unverified |

These estimates use driving radius divided by driven radius and exclude any
motor gearbox reduction. Outside or inside pulley/belt or sprocket radii are not necessarily
pitch radii; exact tooth counts are preferable. Do not multiply these stages
together unless their shaft connections establish that they are in series.

Confirm whether the pivot motor has a gearbox and reconcile the preliminary radius
measurements with the confirmed mechanism layout.
The inferred 4.237:1 pivot chain stage does not by itself establish the complete
motor-to-intake ratio or reconcile the existing 20:1 conversion. No runtime ratio
was changed from these preliminary measurements.

## Units and configuration readiness

Pivot feedback is **intake degrees**, calculated from raw motor rotations:
`degrees = motor rotations × feedback sign × 360 / 20`. The currently configured
ratio is 20:1, so one motor turn corresponds to 18 intake degrees; reconcile this
with the newly reported pivot chain path before treating it as a verified total
reduction. Zero is the verified
upper frame-contact stop; positive degrees lower the intake. Confirmed total
stowed-to-deployed travel is **160 degrees**, so the deployed target is 160.
The real dashboard prefills `IntakeDegrees` and `Pivot/MaxDegrees` with 160;
configuration still requires disabled apply and the remaining hardware settings. The normal feed
position must be greater than zero, clear of the stop, and less than the
lowered intake position.

Shooter readings and targets are **motor RPM**, not wheel RPM. The two motors
have separate targets and tolerances. Each feedback sign is +1 or -1 so a
positive displayed speed means the intended shooting direction. Verify the
relationship among motor inversion, encoder sign and commanded direction on
hardware before confirming directions. Intake rollers and indexer use bounded
open-loop output, so their ratios need not be known yet.

For an unconfigured real robot, `Fuel/Config/PrimaryMotorRpm` and
`Fuel/Config/SecondaryMotorRpm` now start at **2828.57 motor RPM**. This copies
2026-Rebuilt's close-hub motor speed: its 30 wheel RPS preset times 60 times its
22/14 motor-to-wheel ratio. The owner has now confirmed that 16th Note also uses
14T motor pulleys driving 22T flywheel pulleys on both sides. The existing target
therefore corresponds to 1800 flywheel RPM, but remains a provisional starting
point, not a calibrated shot
or a promise of equal wheel speed. Both targets can be tuned independently.
Their checked-in starting values are `INITIAL_SHOOTER_PRIMARY_MOTOR_RPM` and
`INITIAL_SHOOTER_SECONDARY_MOTOR_RPM` in `FuelConstants`. Current limits,
directions, coupling, gains, tolerances and the remaining sequence settings
still require configuration before shooting. Simulation retains its separate
3000 motor-RPM example and trajectory calibration.

`Fuel/Config/` contains staged dashboard settings. They have no effect until
**Fuel/Apply configuration (disabled)** is invoked. The operation rejects enabled
robots, stops all fuel outputs, applies an immutable snapshot, checks controller
results, and invalidates the intake reference. Settings are not written to
controller flash on every iteration. Except for the accepted feed/agitation angles,
dashboard settings are temporary; copy
verified values into `FuelConstants.REAL` using the `FuelConfiguration` records
before relying on them at an event. `Fuel/ActiveConfiguration` displays/logs the
last applied snapshot for review. Restarting restores the checked-in configuration
and reloads the last accepted feed/agitation angles separately.

| Readiness display | Meaning |
| --- | --- |
| `Fuel/DevicesConfigured` / `Fuel/Devices` | All five device configurations succeeded, or missing/failed configuration details |
| `Fuel/Referenced` / `Fuel/Reference` | Pivot reference validity and reference result |
| `Fuel/AutomaticReady` / `Fuel/AutomaticBlockedBy` | Complete configuration, healthy feedback, reference and sequence readiness |
| `Fuel/AutonomousReady` / `Fuel/AutoBlockedBy` | Automatic readiness plus verified auto parameters and measured starting pose |
| `Fuel/ConfigurationParseStatus` | Invalid/missing staged sequence fields after apply |
| `Fuel/ActionStatus` | Result of the last configuration/reference action |

For each role configure `CurrentAmps`, `MaxVolts`, `TestVoltsCap`, `Inverted`,
`FeedbackSign`, `DirectionConfirmed`, `kP`, and `kV`. Current limits and voltage
caps default to zero and block allocation; feedback sign defaults to +1 but
direction is **not** confirmed. Test voltage caps must be positive and no larger
than the normal cap. The validator accepts current limits 1–80 A and voltage
caps up to 12 V; these are validation bounds, **not recommended settings**.
Select appropriate initial values for the actual mechanism before a test.

Choose the shooter coupling explicitly:

- `UNCONFIRMED`: shooter-pair tests and automatic operation are blocked.
- `COUPLED_FOLLOWER`: primary velocity loop with secondary following motor output.
  Confirm compatible equal motor-to-shaft ratios and follower alignment, set
  `FollowerCompatible` and `FollowerInverted`, and use equal motor-RPM targets.
  Both encoder readings remain independently required for readiness.
- `INDEPENDENT`: each shooter has its own velocity loop, gains, target and
  tolerance. Use this only when appropriate to the confirmed mechanical layout.

Pivot positioning uses a roboRIO profiled PID controller, explicit velocity
and acceleration limits, output caps, controller soft limits, and software
travel checks. `Pivot/GravityVolts` is an optional **constant** compensation term,
initially zero; it is not an assumed arm cosine model. Tune against the actual
four-bar load. Device-level pivot `kP/kV` are not the profiled pivot gains;
use `Pivot/kP`, `Pivot/kD`, `Pivot/MaxVelocity`, and `Pivot/MaxAcceleration`.

Automatic sequence settings include measured maximum/intake/feed angles,
position tolerance, both motor-RPM targets/tolerances, stable-ready duration,
spin-up/motion timeouts, intake/indexer duty, and agitation settings. The settings
record validates finite values, travel bounds, timing and output constraints.

## Reference the intake

1. Disable the robot and support/position the intake at its confirmed upper stop:
   the intake plate contacts the frame. Do not force it or use motor power to home.
2. Ensure the pivot has valid electrical configuration and healthy feedback.
3. Invoke **Fuel/Confirm intake at upper stop**.
4. Wait for `Fuel/Reference = Upper reference verified` and `Fuel/Referenced = true`.

The command never moves the motor. It writes encoder zero, verifies the readback
within 0.5 s, and enables applicable soft limits. Late confirmation is rejected;
a failed retry cancels any earlier pending reference. Failed writes, failed soft-limit
configuration or enabling during verification leave the reference invalid.
When deployed travel is still unknown, only the known upper-side soft limit is
established; bounded manual commissioning is available, but automatic motion is
not. An ordinary disabled stop preserves a valid reference. A program restart,
pivot reset, brownout, configuration change or lost pivot communication invalidates
it. Controller resets/brownouts also require a deliberate disabled configuration
reapply. There is no persisted reference and no powered hard-stop homing.

## Test-mode commissioning

LiveWindow actuator operation is disabled; the command scheduler owns Test-mode
commissioning. Its command requires both fuel and drivetrain, continuously
stops the drivetrain, and ends when Test mode ends.

1. While disabled, apply the selected mechanism's electrical limits. Leave
   automatic settings unconfigured until measured.
2. Select `PIVOT`, `INDEXER_PAIR`, or `SHOOTERS` under `Fuel/Test/Mechanism`.
3. Set `Fuel/Test/Volts` to the intended signed test voltage. It defaults to zero.
4. Enable **Test mode**, release RB/R1, then hold RB/R1 to run the test.
5. Release the bumper to stop. Inspect `Fuel/Test/Status` and recorded diagnostics.

The IO clamps requested voltage to the configured test and normal caps. Changing
selection or requested voltage while held requires another release. A lost
controller connection, unhealthy selected mechanism, stale request, mode exit,
or brownout stops the test. A pivot test requires a reference; shooter-pair tests
require confirmed coupling. Direction may remain unconfirmed during low-output
direction tests; mark it confirmed only after checking it. Reapplying a changed
inversion/sign requires another intake reference.

A continuous hold is bounded to **0.25 s for pivot**, **1 s for the indexer pair**,
and **5 s for the shooter pair**. After a burst, release to rearm. The pivot jog
can measure travel before automatic angles or gains exist. Physical travel still
needs operator supervision while the lower limit is unknown. Disabled neutral
output is not an active gravity hold; support the mechanism as necessary.

## Normal driver operation

Xbox/PS4/Logitech Dual Action layout is detected automatically on USB 0. Check
`Drive/ControllerLayout` in any mode, including disabled. To force a layout, set
`TankDriveConstants.AUTO_DETECT_CONTROLLER = false` and
`CONTROLLER_LAYOUT` to `Layout.XBOX`, `Layout.PS4`, or
`Layout.LOGITECH_DUAL_ACTION`, then rebuild/deploy.
Logitech button 5 is intake and button 6 is shoot; button 8 requests vision assist.
Release both bumpers after enable/reconnection/interruption before operating.

| Input | Behavior |
| --- | --- |
| LB / L1 | Lower intake; rollers run while held once deployed |
| Release LB / L1 | Finish lowering, stay down, stop rollers |
| RB / R1 | Spin both shooters; wait for stable readiness, then raise intake and run indexer |
| Release RB / R1 | Stop shooter requests and indexer immediately; return intake fully down |
| Both bumpers | Shooting wins; intake rollers stay off |

A drop in either shooter's speed immediately removes the indexer request and
pauses raising until both recover. Averaging speeds cannot establish readiness.
Releasing shoot returns the intake down even after agitation or during spin-up.
Rollers remain stopped unless LB/L1 is held, and only run once the intake reaches
the deployed target. Disable, faults and motion timeout still stop the return.
Driving and RT/R2 vision assist retain their previous mappings.

Faults and timeouts neutralize fuel outputs and latch until disabled. A controller
connection failure may also invalidate the intake reference. A 100 ms request
watchdog stops abandoned commands; real fuel IO also uses WPILib MotorSafety.
An indexer stop is not proof that gravity-fed fuel cannot reach the shooter.
Validate that behavior physically. The wheels may coast after a stop request.

Automatic bounded agitation is implemented but defaults off. After basic feeding
works, enter measured collision-free intermediate positions, dwells and cycle
count, then enable it through the disabled configuration workflow. There is no
beam break, fuel counter or automatic empty detection.

### Tune how far the intake rises

#### AdvantageScope tuning (recommended for angle adjustments)

1. Run the updated robot program or simulator and connect AdvantageScope using
   **NetworkTables 4 (AdvantageKit)**. Disable real hardware before editing;
   simulation accepts angle edits while enabled.
2. Click the **slider icon beside the sidebar search box** to enable Tuning Mode;
   the icon turns purple.
3. Expand **Tuning → Fuel**. Edit **FeedDegrees** for the highest position the
   intake reaches. Edit **AgitationDegrees** for the intermediate lower position.
   Press Enter or leave the input box to submit each value.
4. Read **AdvantageKit → RealOutputs → Fuel → AngleTuningStatus**. Valid values
   apply automatically while disabled; no dashboard command is needed. The
   corresponding **ActiveFeedDegrees** and **ActiveAgitationDegrees** show the
   accepted targets. On hardware, enabled edits remain pending until disabled. Invalid angles
   leave the last valid settings active and display a rejection reason.
5. Enable teleop and release both bumpers to arm, then hold shoot to test.
   In simulation, edits retarget the active feeding or agitation motion without
   releasing shoot. On hardware, disable before the next adjustment.
   Increase FeedDegrees to stop the intake lower.

These angle-only edits preserve the existing intake reference and do not
reconfigure motor controllers. They do not establish a missing reference or
bypass missing hardware configuration. They affect both teleop and autonomous
feeding targets; autonomous start position and all other settings stay unchanged.
Optional repeated agitation still requires `AgitationEnabled` in the full
configuration. When agitation is off, `AngleTuningStatus` explains that only
`FeedDegrees` affects the shooting position; editing `AgitationDegrees` alone
does not start agitation. Neither field commands motion while idle or disabled.
Adjust the lower endpoint first if raising the feed angle would
otherwise violate the required angle ordering.

Use `/Tuning/Fuel/FeedDegrees` and `/Tuning/Fuel/AgitationDegrees`, not the logged
`NetworkInputs` copies or `RealOutputs`, which are read-only. The new values also
update the staged SmartDashboard angle fields. A later full configuration apply
resynchronizes these tuning inputs and saves valid angles too. The last accepted
pair saves automatically through WPILib's persistent NetworkTables storage and
reloads on startup; angle adjustments need no code edit or redeploy after this
feature is installed. Rejected edits and hardware edits pending disable do not
overwrite the saved pair. For the UI details see
[AdvantageScope Tuning Mode](https://docs.advantagescope.org/overview/live-sources/tuning-mode/).

The saved pair is stored under `/Preferences/16thNote/Fuel/SIM/IntakeAngles` or
`/Preferences/16thNote/Fuel/REAL/IntakeAngles`. SIM and REAL values are independent;
replay does not save angles. The server writes persistent values asynchronously,
so allow a few seconds after an accepted edit before shutting down. In desktop
simulation, keep the project's ignored `networktables.json` file across restarts.
The roboRIO keeps its own persistent file outside the deployed code. See
[WPILib persistent topics](https://docs.wpilib.org/en/latest/docs/software/networktables/networktables-intro.html).

Saved angles are revalidated against the current configuration. If hardware is
unconfigured, they populate the staged angle fields but cannot enable motion or
replace the required hardware configuration. Startup defaults are used when no
saved pair exists; invalid saved targets are rejected. Intake referencing and the
agitation enable setting are not persisted by this feature.

If an accepted angle does not move the simulated intake:

- Keep the robot simulator running; AdvantageScope alone does not run the robot code.
- While disabled, use **Fuel/Confirm intake at upper stop** to establish the
  simulated reference. Enable Teleop, release both bumpers, then hold shoot.
- For repeated agitation, set **Fuel/Config/AgitationEnabled** to true and run
  **Fuel/Apply configuration (disabled)**, then confirm the reference again.
  The agitation endpoint must be between the feed and deployed positions, with
  clearance for the position tolerance. Equal feed/agitation angles are invalid
  when agitation is enabled.
- The simulation defaults to three agitation cycles per shooting hold. Release
  and press shoot again after those cycles to test another agitation endpoint.
- Edit the values in the AdvantageScope tuning fields themselves. While tuning
  is active, AdvantageScope republishes its entered values and can overwrite
  edits sent by another NetworkTables client.

The agitation enable setting is temporary and must be reapplied after restarting
the simulator. Check **Fuel/AngleTuningStatus**, **Fuel/State**, and
**Fuel/AutomaticBlockedBy** to distinguish accepted targets from blocked motion.

#### Full configuration workflow in Shuffleboard

The upper position used for feeding and the upward part of agitation is already
tunable through **Fuel/Config/FeedDegrees**. **Fuel/Config/AgitationDegrees**
is the intermediate lower position between raises, not the upper endpoint.

| Dashboard setting | What it controls |
| --- | --- |
| `Fuel/Config/FeedDegrees` | Raised feeding position, also the upper agitation endpoint |
| `Fuel/Config/AgitationDegrees` | Intermediate lower endpoint during optional agitation |
| `Fuel/Config/IntakeDegrees` | Fully deployed pickup/return position (160 degrees) |

Zero is fully up at the reference stop; larger angles are farther down. If the
intake rises too high, **increase FeedDegrees** in small increments. This also
changes the initial feeding position when shooting. When agitation is enabled,
keep its intermediate angle between the feed and deployed angles, separated
from each endpoint by more than `PositionToleranceDegrees`. If increasing
FeedDegrees would pass AgitationDegrees, adjust both endpoints while preserving
that ordering. The correct working angles must be established on the mechanism.

1. Disable the robot and edit the staged angle(s) in Shuffleboard/SmartDashboard.
2. Invoke **Fuel/Apply configuration (disabled)**. Editing a field alone does not
   change the active targets. Check `Fuel/ActionStatus` and
   `Fuel/ConfigurationParseStatus` for the result.
3. Applying the full configuration invalidates the reference. Put the intake at
   its actual upper mechanical stop and run **Fuel/Confirm intake at upper stop**;
   do not zero it at the new working angle. Follow the referencing procedure above.
4. Enable teleop, release both bumpers to arm, then hold shoot to test the raised
   position. Optional repeated agitation still requires `AgitationEnabled`.
5. Disable between adjustments. Accepted feed/agitation angles save automatically
   across restarts. Other dashboard configuration values remain temporary.

## Autonomous: back up and shoot preloads

**Do nothing** remains the default chooser option. The optional routine requires
both drivetrain and fuel, and checks readiness before either subsystem moves.
The real auto fields default unverified/unset:

- `Auto/Verified`: affirm the preload procedure, starting pose and backing test.
- `Auto/StartDegrees`: expected intake angle; feedback must be within tolerance.
- `Auto/LeftVolts`, `Auto/RightVolts`: verified negative backing voltages.
- `Auto/BackSeconds`, `Auto/SettleSeconds`, `Auto/FeedSeconds`, `Auto/TotalSeconds`.

The sequence holds the starting intake pose while backing, stops the drivetrain,
settles, spins up, then feeds when both motors are ready. Feeding duration begins
when indexer output first becomes positive and continues through speed-recovery
pauses. A separate overall timeout is limited to 15 s. Configuration/feedback
failure, missing reference, disable, cancellation or timeout stops both subsystems.
See `Fuel/Auto/Stage` and `Fuel/Auto/Reason`.

Backing is voltage-and-time based. It does not measure distance or correct heading;
wheel slip, carpet and battery conditions affect the result. Measure and validate
parameters at the actual starting placement. The preload intake position is still
a mechanical decision; do not check `Auto/Verified` until it has been tested.

## Simulation, tests and telemetry

Use the WPILib 2026 JDK from `16th_note-Imported/`:

```sh
./gradlew build
./gradlew simulateJava
```

Enable the optional simulation GUI in WPILib for interactive joystick input.
SIM exposes independent shooter jams/connections, pivot jams, stale feedback,
reference loss and indexer disconnection under `Fuel/Sim/`. Production simulation
starts unreferenced; use the disabled reference action before mechanism operation.
The mechanism model does not validate torque, gravity loads, collision clearance,
shooting accuracy or real gearing. A separate SIM-only practice world renders
pickup, six-ball inventory, ballistic shots and approximate hub entry; see below. Its configuration can be exercised before CAD
measurements exist, but must not be copied to REAL as calibrated settings.

AdvantageKit `Fuel/Inputs` logs per-device configuration/connection flags, raw motor
rotations/RPM, voltage/current/temperature, reset/brownout/fault indications,
reference state, requests and timestamps. Array order is primary shooter, secondary
shooter, primary indexer, secondary indexer, then pivot (CAN 3–7).
Commands, state and fault reasons are
recorded under `Fuel/`. Controller read success and the 100 ms periodic-frame timeout
are health evidence; `ageSeconds` is a successful-read gate, **not a timestamp of a
newly received CAN frame**. There is no fabricated freshness measurement.

Telemetry now uses `ShooterPrimaryMotorRpm` and `ShooterSecondaryMotorRpm`; old
mechanism-RPM logs and old schema are not interchangeable. Replay allocates no real
motors; end-to-end replay of configuration actions/commissioning is not validated.
Use the matching reviewed configuration when interpreting logs.

Tests cover real-IO decisions through fake motor ports, failed configuration/zeroing,
reference invalidation and conversion, soft limits, both shooter modes, separate RPM
readiness, commissioning interlocks, and the simulated full autonomous sequence.
These tests do not prove the physical robot is commissioned.

## Ordered hardware handoff

1. Verify the five mechanism devices and CAN IDs 3–7. Select conservative mechanism-appropriate
   current and test-output limits; apply disabled and confirm stop behavior.
2. Establish the pivot reference. Check inversion/feedback sign with bounded jogs,
   re-reference after configuration changes, then measure deployed travel and a
   raised working position clear of the hard stop. Configure soft/profile limits.
3. Check roller and indexer directions separately. Verify that stopped indexing
   prevents unwanted feeding and that loaded fuel does not jam the stopped shooter.
4. Confirm shooter coupling/follower alignment. Test the pair, tune motor-RPM gains,
   targets and tolerances, then verify readiness under fuel load.
5. Test pickup, intake release, spin-up, feed, shoot release, disable and recovery
   with three fuel first, then the full six-fuel capacity and realistic battery load.
6. Verify autonomous starting pose and backing/settling/feed timings. Rehearse the
   entire routine and then explicitly mark the auto configuration verified.
7. Tune optional agitation only after the basic cycle works reliably.
8. Copy reviewed settings into REAL configuration, rebuild, and repeat a startup,
   reference and full-cycle test. Keep a known-good build and configuration available.

Target the complete rehearsal for October 13; reserve October 15–16 for fixes before
travel to the October 17–18 event. Do not deploy the simulation examples as robot tuning.


## AdvantageScope practice world

This runs only in SIM, never on the roboRIO or during replay. It uses the same
fuel sequence and driver mappings as the robot. The rigid CAD does not animate;
a separate Mechanism2d overlay shows estimated intake motion. Use the project's
`sim` folder as the AdvantageScope custom-assets parent and choose **16th Note**.

1. From `16th_note-Imported`, run `./gradlew simulateJava`. Enable the simulation
   GUI in the WPILib simulation-extension selector for controller and mode input
   (the Gradle default has the GUI off). Assign your controller to USB 0.
2. In AdvantageScope choose **File > Connect to Simulator > NetworkTables 4
   (AdvantageKit)**. A running local simulator is required; importing CAD does
   not start it. Select **FRC:2026 Field**, with the blue-origin coordinate system.
3. Remove the crossed-out `RobotRemy` and `FieldSimulation` sources. Drag the new
   fields below from the live data tree into the 3D Field's Poses list. Names here
   are logger keys; NetworkTables adds `/AdvantageKit/RealOutputs/` in SIM too.

| Logger key | AdvantageScope object |
| --- | --- |
| `Visualization/16thNote/RobotPose` | Robot; select the 16th Note model |
| `Visualization/16thNote/Intake` | Mechanism child of that robot, XZ plane |
| `Simulation/Fuel/GroundPoses` | Fuel game pieces |
| `Simulation/Fuel/ProjectilePoses` | Fuel game pieces |
| `Simulation/Fuel/HeldPoses` | Fuel game pieces |

Plot `Simulation/Fuel/HeldCount`, `LaunchedCount`, and `ScoredCount` or view the
SmartDashboard `Fuel/Sim/` counters. These are simulation truth only, not real
sensor estimates. `ScoredCount` counts entries into either hub; it is not match
points and does not implement active/inactive hub periods.

While disabled, use **Fuel/Sim/Reset empty (disabled)** for pickup practice or
**Fuel/Sim/Reset six preloads (disabled)** for shooting. Both explicitly reset the drive pose, field balls, counters and projectiles, and
place/reference the **simulated** intake at its upper stop. Startup itself still
requires a reference or an explicit practice reset; real referencing is unchanged. Enable
Teleop, release bumpers, then use the existing drive/LB/RB controls. The six-preload reset turns the robot so its rear shooter faces the blue hub; 400 ground fuel are arranged in a non-overlapping 20-by-20 grid centered in the
neutral zone. This is a repeatable practice layout, not a claim of exact official
match staging. Six preloads, when selected, are additional to the 400 ground fuel.
Keep the pose unchanged to try the six-preload shot with the example settings.

For the existing autonomous example, the starting intake angle must still match
`Fuel/Config/Auto/StartDegrees`. One simulated exercise starts at the upper stop:
while disabled set that dashboard value to 0, apply configuration, confirm the
upper reference, reset six preloads, then select **Back up and shoot preloads**
and enable Autonomous. This is a SIM-only example, not a verified real preload
position. These autonomous dashboard settings are temporary.

`Fuel/Sim/LaunchSpeedMps` (default approximately 6.39 at 3000 motor RPM) is a practice estimate.
The hood angle is fixed at **80 degrees above horizontal**, matching the supplied
mechanical constraint; it is not dashboard-tunable. Shooter motor
RPM scales the example exit speed; no wheel ratio, compression or efficiency is
inferred. The muzzle height/offset, pickup area, intake overlay and hub aperture
are estimates too. A downward crossing of the approximate hub opening counts;
misses land and can be collected again. Balls already in flight keep falling
when disabled, but new launches and pickup stop. Test mode cannot collect/shoot.
The drivetrain now stops against approximate field walls and square hub footprints
using a conservative circular robot envelope. It can rotate and back away after
contact. This is a flat collision constraint, not an impact/traction model. There
are no ramp/trench/tower, other-robot or ball collisions, bounce, drag, drivetrain
traction calibration, vision detections, or jam physics. Do not use this world to decide
real mechanism limits or shooter settings.

AdvantageScope formats and live connection instructions:
https://docs.advantagescope.org/tab-reference/3d-field/
https://docs.advantagescope.org/overview/live-sources/


### Crossed-out fuel sources or intake that will not move

Fuel sources must use `Pose3d[]` (an array), not `Pose3d`. Delete wrongly typed
sources and drag the parent fields marked **Pose3d[]** from the live data tree.
The intake overlay is the field **Intake – Mechanism2d**; add it as a child of the
robot, not as a CAD Component. No separate intake GLB is needed for simulated pickup.
If `Fuel/Reference` says `Not referenced`, disable the simulator and invoke
**Fuel/Confirm intake at upper stop**, then release both bumpers after re-enabling.
Reference loss still blocks mechanisms deliberately; do not bypass it in real code.


### One-button pickup practice

While the simulator is **disabled**, invoke **Fuel/Sim/Prepare pickup practice
(disabled)** from a command-capable dashboard (SmartDashboard or Shuffleboard).
This places the robot beside the first row of neutral-zone fuel, resets the
400 ground balls and empty inventory, and references the simulated intake.
Then enable **Teleop**, release both bumpers once, hold LB/L1 until the intake
lowers, and drive forward. The count should reach six and stop increasing.
Releasing LB/L1 stops collection. This does not configure or reference real hardware.

Watch **Fuel/Sim/PickupStatus** for the reason pickup is blocked. **PickupActive**
means the rollers are running at the deployed intake position, **GroundCount**
starts at 400, and **NearestFuelMeters** is distance from the modeled pickup mouth.
Add `Simulation/Fuel/PickupMouth` as an **Axes** object (`Pose3d`) in AdvantageScope
if the CAD's apparent front is unclear. Pickup follows robot +X, independent of
CAD orientation; the marker shows the actual collection location. Fuel pose
arrays still use the same three `Pose3d[]` fields, so no layout changes are needed.
Pickup sweeps between adjacent active steps to avoid skipping fuel while driving.
Mode changes, inactive intake, and resets break the sweep; they never collect
fuel along a teleport. A practice reset is rejected while enabled.


### Shooter direction and speed in simulation

The intake collects on robot **+X** and the shooter exits on robot **-X**.
Shots originate behind the robot center and follow the opposite direction to
its heading. The six-preload reset faces the rear shooter toward the blue hub from a static
starting distance calculated for the example 6.39 m/s, 80-degree trajectory;
the pickup reset still faces the intake toward the neutral-zone fuel.

`Fuel/Sim/LaunchSpeedMps` controls estimated ball exit speed at 3000 motor RPM;
The fixed 80-degree hood angle is used at every speed. Increasing exit speed
increases both height and horizontal range and can cause overshooting. These values do not tune
real flywheel gains or RPM. The current example takes approximately 0.75 s to
reach 3000 RPM, then requires 0.2 s of stable readiness before feeding; balls
are released at most once every 0.3 s. Those are separate timing assumptions.

The faster ball-speed example is a software practice value. Its preload reset is
farther from the hub than the old 5.8 m/s example. It does not specify where the
real fixed-hood robot should shoot; actual close-hub speed and geometry still need
measurement. Changing LaunchSpeedMps does not automatically move the robot or
retune the hood, and can cause a miss. Pickup direction and driver controls are
unchanged.

`Fuel/Sim/Hood` describes the fixed angle. An old `LaunchAngleDegrees` widget may
remain in a saved dashboard; it is no longer read by the simulator and can be
removed. The model assumes the fuel exits at the stated hood angle; actual exit
speed and trajectory still require hardware measurements.


## Detailed CAD intake animation

`sim/Robot_16thNoteDetailed` contains the combined October 5 drive-base and
shooter meshes plus a separate movable intake mesh. Select **16th Note Detailed**
in AdvantageScope, then add
`NT:/AdvantageKit/RealOutputs/Visualization/16thNote/ComponentPoses` as a
**Component** child of that robot. The source type must be **Pose3d[]**.
The older `Intake` Mechanism2d source is an optional stick diagram; remove that
child if you only want the detailed intake. Restart simulation to publish the
new topic. On another laptop, copy/use this repository's `sim` folder as the
custom asset folder; do not use a stale copy from the REBUILT repository.

The user confirmed the CAD pose is lowered for pickup. The animation rotates
the intake plates/roller/backer/outer belt around the shared jackshaft axis;
fixed motor-to-shaft belts remain stationary. The animation uses the confirmed **160-degree stowed-to-deployed travel**.
At zero it is stowed; at 160 it matches the lowered CAD export. The SIM feed
target remains 5 degrees clear of the upper stop and still requires physical
validation. Verify mounting alignment before treating the model as a
physical-motion reference. `FuelVisualization` holds only visualization
offsets; no real controller configuration, pickup physics, or shooter ballistics
are derived from these meshes. See the asset README for geometry assumptions.


## Nine-foot shot-height target

The current target is **9 feet from carpet to the top of the fuel ball**, not
its center. `Fuel/Sim/PeakLimitFeet` defaults to 9 and can be increased later
when venue clearance and shooting measurements justify it. This is a temporary
SIM-only setting, restored on restart. `Fuel/Sim/PredictedPeakFeet` reports the
ballistic prediction and `Fuel/Sim/ShotBlockedBy` reports rejected shots.

The default SIM launch speed is approximately **6.39 m/s at 3000 motor RPM**,
calculated for the fixed 80-degree launch angle, estimated 0.65 m release height,
0.075 m ball radius and gravity alone. The corresponding descending shot reaches
the hub center from about **1.17 m / 3.84 ft**, measured horizontally from the
release point, not from the bumper. The six-preload reset uses this starting
range. Changing the height limit does not automatically change speed or position.

A predicted over-height shot or invalid limit prevents a new simulated launch
and does not consume simulated inventory. Actual velocity is not silently
clamped, and projectiles already in flight continue on their original trajectories.
The check includes the motor-RPM scaling, so increasing either launch speed or
motor RPM cannot bypass the simulated height limit.

**This does not enforce a nine-foot ceiling on the real robot.** The real
2828.57 motor-RPM starting targets are not calibrated to ball exit speed. Measure
release height, launch angle, peak height and successful shot distance on hardware,
then build a verified distance-to-RPM table with margin below the height target.
Air resistance, spin and shot variation are not modeled here. Automatic
selection of flywheel speed from distance remains pending that calibration and
reliable distance measurement; the current robot still uses fixed RPM targets.


## Optional trench practice shots (SIM only)

Close shooting remains the default with a 9-foot height limit. Optional fixed
left/right trench practice locations are on the **blue alliance side**, clear
of the trench structure; left/right are as viewed from the blue driver stations.
They are practice estimates, not measured shooting marks or autonomous paths.
At the fixed 80-degree hood angle, both are approximately 10.6 ft from release
to hub center and predict an 18.4 ft ball-top peak. Their provisional SIM target
is approximately 4666 motor RPM. These RPM values are NOT calibrated real targets.

1. Disable simulation.
2. Set `Fuel/Sim/TrenchShotsEnabled` to true (it starts false).
3. Run **Fuel/Sim/Prepare left trench shot (disabled)** or
   **Fuel/Sim/Prepare right trench shot (disabled)**.
4. The command sets both SIM motor targets, restores the standard exit-speed
   conversion, sets the peak limit to 19 ft, places the robot facing the hub,
   loads six fuel, and references the simulated intake. It preserves the other
   active mechanism settings. Enable Teleop, release bumpers, then hold RB/R1.
5. To return to close shooting, disable and run **Fuel/Sim/Reset six preloads
   (disabled)**. It restores 3000 SIM motor RPM and the 9-foot limit, repositions
   the robot and turns off trench shots.

Turning `TrenchShotsEnabled` off while a trench preset is active immediately
blocks new simulated launches, but does not remove balls already airborne or
change the active shooter motor request. Release RB/R1 to stop those requests.
The pickup/empty reset commands keep the current shot settings; use the close
six-preload reset to restore close settings explicitly. `Fuel/Sim/ShotPreset`
shows the last selected shot preset. All profile changes are rejected while
enabled, and the preset-target API rejects any non-simulation IO backend.

The reported venue ceiling estimate is 20–30 ft, not a verified lowest-obstruction
measurement. A 19-foot planned cap leaves only about 1 ft beneath an assumed
20-foot obstruction. Before enabling a corresponding real trench shot, verify
lights/beams and field placement, then measure the actual trajectory and RPM
with suitable margin. There is no real trench preset or automatic range-to-RPM
selection enabled by these practice controls. Robot hardware calibration and
real autonomous remain governed by their existing checks.
