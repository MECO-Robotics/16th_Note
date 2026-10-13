# 16th Note Roboticon Checklist

Check off each task by changing `[ ]` to `[x]`. Complete the sections in order.
PhotonVision, AprilTag localization, distance-based shooting, and trench shooting
are outside the Roboticon scope.

## 1. Electrical and controller setup

- [ ] Program SPARK MAX CAN IDs 1–7.
- [ ] Verify controller firmware and confirm that every CAN device communicates.
- [ ] Inspect all power, CAN, motor, and encoder wiring.
- [ ] Connect the competition controller on Driver Station USB 0.
- [ ] Verify the detected controller layout and every button mapping.

## 2. Deploy and drivetrain checks

- [ ] Deploy the latest `main` build to the roboRIO.
- [ ] Lift the drivetrain so the wheels can spin safely.
- [ ] Verify that a forward command turns both sides forward.
- [ ] Verify left and right drivetrain inversion.
- [ ] Test forward, reverse, left turn, right turn, disable, and controller disconnect.

## 3. Initial fuel configuration

- [ ] Enter conservative current limits for all five fuel motors.
- [ ] Enter conservative normal-voltage and test-voltage caps.
- [ ] Determine the inversion for each fuel motor.
- [ ] Determine the encoder feedback sign for each fuel motor.
- [ ] Select the verified shooter coupling configuration.
- [ ] Apply the configuration while disabled.
- [ ] Confirm all five fuel controllers report configured and healthy.

## 4. Low-output mechanism checks

- [ ] Test `INDEXER_PAIR` at low voltage.
- [ ] Confirm indexer motors 5 and 6 assist each other instead of fighting.
- [ ] Confirm the linked intake roller turns in the collection direction.
- [ ] Test both shooters at low output.
- [ ] Confirm both flywheels turn in the shooting direction.
- [ ] Support the intake so it cannot fall while testing the pivot.
- [ ] Test the pivot with short, low-voltage jogs.
- [ ] Confirm positive pivot feedback corresponds to lowering the intake.

## 5. Pivot reference and motion

- [ ] Place the intake against its upper frame-contact stop while disabled.
- [ ] Run **Fuel/Confirm intake at upper stop**.
- [ ] Confirm `Fuel/Referenced` is `true`.
- [ ] Confirm `Fuel/Reference` reports `Upper reference verified`.
- [ ] Verify one pivot-motor rotation changes the angle by approximately 46.3158 degrees.
- [ ] Verify the actual stowed-to-deployed travel is 160 degrees.
- [ ] Verify the upper and lower software limits stop motion correctly.
- [ ] Tune pivot `kP` and `kD`.
- [ ] Tune pivot gravity voltage.
- [ ] Tune pivot maximum velocity and acceleration.
- [ ] Confirm the pivot reaches and holds each target without oscillating or dropping.

## 6. Intake and indexer tuning

- [ ] Tune the intake collection duty cycle.
- [ ] Tune the indexer feeding duty cycle.
- [ ] Tune the fully deployed intake position.
- [ ] Tune the normal feed position.
- [ ] Confirm the intake roller stops when collection is released.
- [ ] Confirm the indexer stops immediately when shooting is released.
- [ ] Leave automatic agitation disabled until normal feeding is reliable.

## 7. Shooter tuning

- [ ] Tune the primary shooter velocity gains.
- [ ] Tune the secondary shooter velocity gains.
- [ ] Find a reliable close-range shooter RPM.
- [ ] Tune each shooter's RPM tolerance.
- [ ] Tune the stable-ready time.
- [ ] Confirm feeding starts only when both shooters are ready.
- [ ] Confirm a speed drop stops feeding immediately.
- [ ] Verify the shot remains below the event height limit.

## 8. Full mechanism validation

- [ ] Run complete intake-to-shoot cycles with three fuel.
- [ ] Run complete intake-to-shoot cycles with the full six-fuel capacity.
- [ ] Test controlled jam recovery.
- [ ] Disable while the intake is moving and confirm all powered motion stops.
- [ ] Disable while shooting and confirm all commands stop safely.
- [ ] Test with a realistic battery load.
- [ ] Confirm there are no roboRIO brownouts or SPARK MAX resets.
- [ ] Review AdvantageKit current, voltage, RPM, pivot-position, and fault logs.

## 9. Save the REAL configuration

- [ ] Copy every verified dashboard setting into `FuelConstants.REAL`.
- [ ] Build and deploy the checked-in REAL configuration.
- [ ] Restart the robot program.
- [ ] Confirm all fuel devices configure without re-entering dashboard values.
- [ ] Reference the intake again after restart.
- [ ] Confirm saved feed and agitation angles reload correctly.
- [ ] Repeat one complete intake-to-shoot cycle after restart.

## 10. Autonomous decision

- [ ] Decide whether Roboticon will use **Do nothing** or the simple preload autonomous.
- [ ] Keep **Do nothing** selected until the preload routine is completely verified.
- [ ] If using preload autonomous, verify the starting position.
- [ ] If using preload autonomous, tune backing and settling behavior.
- [ ] If using preload autonomous, tune spin-up and feed timing.
- [ ] If using preload autonomous, run repeated full autonomous tests.
- [ ] Confirm disable, missing reference, shooter-speed loss, and timeout stop autonomous.

## 11. Final software release

- [ ] Run the complete automated test suite.
- [ ] Confirm every test passes.
- [ ] Review the final Git diff for unintended files or settings.
- [ ] Commit the final event configuration.
- [ ] Push the final commit to GitHub.
- [ ] Record or tag the exact commit deployed to the robot.
- [ ] Keep a copy of the known-good deployable build.

## 12. Event readiness

- [ ] Test, charge, and label competition batteries.
- [ ] Verify radio power, mounting, and Driver Station connection.
- [ ] Check bumpers, robot weight, and event inspection requirements.
- [ ] Check chain and belt tension, gear alignment, fasteners, and retaining hardware.
- [ ] Prepare spare motors, controllers, chain, belts, wire, connectors, and fasteners.
- [ ] Print or save an offline copy of the CAN map and startup procedure.
- [ ] Practice deployment, connection, configuration, and intake referencing.
- [ ] Practice driving, collecting, shooting, disabling, and jam recovery.
- [ ] Complete a final full-system test using the event battery and controller.
