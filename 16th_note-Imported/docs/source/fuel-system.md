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
motor is a brushless NEO controlled by a SPARK MAX:

| CAN ID | Configuration role | Mechanism |
| --- | --- | --- |
| 3 | `SHOOTER_PRIMARY` | Primary shooter motor |
| 4 | `PIVOT` | Intake movement |
| 5 | `ROLLERS` | Intake rollers |
| 6 | `SHOOTER_SECONDARY` | Secondary shooter motor |
| 7 | `INDEXER` | Independently controlled indexer |

Assign these IDs in REV Hardware Client; the robot code does not change IDs.
Intake rollers and indexer are independently powered but coaxial. Shooter
shaft coupling, directions and ratios still require confirmation.

## Units and configuration readiness

Pivot feedback is **intake degrees**, calculated from raw motor rotations:
`degrees = motor rotations × feedback sign × 360 / 20`. The confirmed ratio is
20:1, so one motor turn corresponds to 18 intake degrees. Zero is the verified
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
22/14 motor-to-wheel ratio. That ratio belongs to REBUILT, not 16th Note; the
result is only a provisional motor-speed starting point, not a calibrated shot
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
controller flash on every iteration. Dashboard settings are temporary; copy
verified values into `FuelConstants.REAL` using the `FuelConfiguration` records
before relying on them at an event. `Fuel/ActiveConfiguration` displays/logs the
last applied snapshot for review. Restarting restores the checked-in defaults.

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
2. Select `PIVOT`, `ROLLERS`, `INDEXER`, or `SHOOTERS` under `Fuel/Test/Mechanism`.
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

A continuous hold is bounded to **0.25 s for pivot**, **1 s for rollers/indexer**,
and **5 s for the shooter pair**. After a burst, release to rearm. The pivot jog
can measure travel before automatic angles or gains exist. Physical travel still
needs operator supervision while the lower limit is unknown. Disabled neutral
output is not an active gravity hold; support the mechanism as necessary.

## Normal driver operation

Select Xbox or PS4 with `TankDriveConstants.USE_XBOX_CONTROLLER`, on USB 0.
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
reference state, requests and timestamps. Array order is primary shooter, pivot,
rollers, secondary shooter, indexer (CAN 3–7). Commands, state and fault reasons are
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

1. Verify five devices and CAN IDs. Select conservative mechanism-appropriate
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
position. Dashboard settings are temporary.

`Fuel/Sim/LaunchSpeedMps` (default 7.5 at 3000 motor RPM) is a practice estimate.
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
starting distance calculated for the example 7.5 m/s, 80-degree trajectory;
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
