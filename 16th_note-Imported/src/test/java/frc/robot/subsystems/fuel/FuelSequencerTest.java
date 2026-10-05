package frc.robot.subsystems.fuel;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.constants.fuel.FuelConstants.Settings;
import frc.robot.sim.fuel.FuelIOSim;
import frc.robot.subsystems.fuel.FuelSequencer.Feedback;
import frc.robot.subsystems.fuel.FuelSequencer.Output;
import frc.robot.subsystems.fuel.FuelSequencer.State;
import java.util.EnumSet;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

class FuelSequencerTest {
  private final Settings settings = FuelTestSettings.EXAMPLE;

  private Feedback feedback(double degrees, double rpm) {
    return new Feedback(true, true, true, degrees, rpm, rpm, 0);
  }

  private FuelSequencer armed() {
    FuelSequencer sequence = new FuelSequencer(settings);
    sequence.update(0, true, true, false, false, false, feedback(settings.feedDegrees(), 0));
    assertEquals(State.IDLE, sequence.state());
    return sequence;
  }

  private Output tick(
      FuelSequencer sequence,
      double time,
      boolean intake,
      boolean shoot,
      double degrees,
      double rpm) {
    return sequence.update(time, true, true, false, intake, shoot, feedback(degrees, rpm));
  }

  @Test
  void heldButtonsCannotStartAtEnableOrAfterControllerReconnect() {
    FuelSequencer sequence = new FuelSequencer(settings);
    assertEquals(Output.stopped(), tick(sequence, 0, true, true, 70, 0));
    assertEquals(State.WAIT_FOR_RELEASE, sequence.state());
    tick(sequence, 0.02, false, false, 70, 0);
    assertTrue(tick(sequence, 0.04, true, false, 70, 0).pivotEnabled());
    assertEquals(
        Output.stopped(), sequence.update(0.06, true, false, false, true, false, feedback(60, 0)));
    assertEquals(Output.stopped(), tick(sequence, 0.08, true, false, 60, 0));
    tick(sequence, 0.1, false, false, 60, 0);
    assertTrue(tick(sequence, 0.12, true, false, 60, 0).pivotEnabled());
  }

  @Test
  void intakeStaysDownAfterReleaseAndRollersOnlyRunWhenDownAndHeld() {
    var sequence = armed();
    var output = tick(sequence, 0.02, true, false, 70, 0);
    assertEquals(settings.intakeDegrees(), output.pivotDegrees());
    assertEquals(0, output.intakeDuty());
    output = tick(sequence, 0.2, false, false, 30, 0);
    assertEquals(settings.intakeDegrees(), output.pivotDegrees());
    assertEquals(State.DEPLOYING, sequence.state());
    tick(sequence, 0.4, false, false, 0, 0);
    assertEquals(State.DEPLOYED, sequence.state());
    output = tick(sequence, 0.42, true, false, 0, 0);
    assertEquals(settings.intakeDutyCycle(), output.intakeDuty());
    output = tick(sequence, 0.44, false, false, 0, 0);
    assertEquals(0, output.intakeDuty());
    assertEquals(0, output.pivotDegrees());
    assertEquals(State.DEPLOYED, sequence.state());
  }

  @Test
  void shooterWinsBothButtonsAndRequiresContinuousReadyTime() {
    var sequence = armed();
    var output = tick(sequence, 0.02, true, true, 0, 0);
    assertEquals(State.SPINNING_UP, sequence.state());
    assertEquals(settings.shooterPrimaryMotorRpm(), output.shooterPrimaryMotorRpm());
    assertEquals(0, output.intakeDuty());
    assertEquals(0, output.pivotDegrees());
    tick(sequence, 0.2, true, true, 0, 3000);
    tick(sequence, 0.3, true, true, 0, 2400);
    tick(sequence, 0.4, true, true, 0, 3000);
    tick(sequence, 0.5, true, true, 0, 3000);
    assertEquals(State.SPINNING_UP, sequence.state());
    output = tick(sequence, 0.61, true, true, 0, 3000);
    assertEquals(State.FEEDING, sequence.state());
    assertEquals(settings.feedDegrees(), output.pivotDegrees());
  }

  @Test
  void droppingSpeedPausesPivotThenRequiresStableRecovery() {
    var sequence = armed();
    tick(sequence, 0.02, false, true, 0, 3000);
    tick(sequence, 0.23, false, true, 0, 3000);
    var output = tick(sequence, 0.25, false, true, 20, 2000);
    assertEquals(State.SPINNING_UP, sequence.state());
    assertEquals(20, output.pivotDegrees());
    // The hold target must not creep along with a coasting arm while waiting for speed.
    output = tick(sequence, 0.27, false, true, 21, 2100);
    assertEquals(20, output.pivotDegrees());
    tick(sequence, 0.3, false, true, 20, 3000);
    output = tick(sequence, 0.51, false, true, 20, 3000);
    assertEquals(settings.feedDegrees(), output.pivotDegrees());
  }

  @Test
  void releasingShootStopsWheelAndHoldsWithoutRetractingEvenIfIntakeStillHeld() {
    var sequence = armed();
    tick(sequence, 0.02, true, true, 0, 3000);
    tick(sequence, 0.23, true, true, 0, 3000);
    var output = tick(sequence, 0.25, true, false, 25, 3000);
    assertEquals(State.IDLE, sequence.state());
    assertEquals(0, output.shooterPrimaryMotorRpm());
    assertEquals(0, output.intakeDuty());
    assertEquals(25, output.pivotDegrees());
    output = tick(sequence, 0.27, true, false, 25, 2800);
    assertEquals(25, output.pivotDegrees());
    tick(sequence, 0.29, false, false, 25, 2600);
    output = tick(sequence, 0.31, true, false, 25, 2400);
    assertEquals(settings.intakeDegrees(), output.pivotDegrees());
  }

  @Test
  void spinupAndMotionTimeoutsLatchUntilDisabled() {
    var sequence = armed();
    tick(sequence, 0.02, false, true, 0, 0);
    assertEquals(Output.stopped(), tick(sequence, 4.03, false, true, 0, 0));
    assertEquals(State.FAULT, sequence.state());
    assertTrue(sequence.fault().contains("Shooter"));
    sequence.stop();
    assertEquals(Output.stopped(), tick(sequence, 4.05, false, false, 0, 3000));
    assertEquals(State.FAULT, sequence.state());
    sequence.update(4.1, false, true, false, false, false, feedback(0, 0));
    assertEquals("", sequence.fault());
    tick(sequence, 4.12, false, false, 70, 0);
    tick(sequence, 4.14, true, false, 70, 0);
    assertEquals(Output.stopped(), tick(sequence, 7.15, true, false, 70, 0));
    assertTrue(sequence.fault().contains("Intake motion"));
  }

  enum Failure {
    DISCONNECT,
    UNREFERENCED,
    STALE,
    NAN_POSITION,
    NAN_SPEED,
    OUT_OF_TRAVEL,
    BROWNOUT
  }

  @ParameterizedTest
  @EnumSource(Failure.class)
  void invalidFeedbackAndBrownoutStopAllOutputs(Failure failure) {
    var sequence = armed();
    tick(sequence, 0.02, true, false, 0, 0);
    Feedback feedback =
        switch (failure) {
          case DISCONNECT -> new Feedback(true, false, true, 0, 0, 0, 0);
          case UNREFERENCED -> new Feedback(true, true, false, 0, 0, 0, 0);
          case STALE -> new Feedback(true, true, true, 0, 0, 0, 1);
          case NAN_POSITION -> feedback(Double.NaN, 0);
          case NAN_SPEED -> feedback(0, Double.NaN);
          case OUT_OF_TRAVEL -> feedback(100, 0);
          case BROWNOUT -> feedback(0, 0);
        };
    assertEquals(
        Output.stopped(),
        sequence.update(0.04, true, true, failure == Failure.BROWNOUT, true, true, feedback));
    assertEquals(State.FAULT, sequence.state());
    assertFalse(sequence.fault().isBlank());
    assertEquals(Output.stopped(), tick(sequence, 0.06, false, false, 0, 0));
  }

  @Test
  void realUnconfiguredBackendNeverProducesOutputs() {
    var sequence = new FuelSequencer(settings);
    var unavailable = new Feedback(false, false, false, 0, 0, 0, 0);
    assertEquals(
        Output.stopped(), sequence.update(0, true, true, false, false, false, unavailable));
    assertEquals(Output.stopped(), sequence.update(1, true, true, false, true, true, unavailable));
    assertEquals(State.UNAVAILABLE, sequence.state());
  }

  @Test
  void disabledStopsAllOutputsAndRequiresReleaseBeforeRestart() {
    var sequence = armed();
    tick(sequence, 0.02, true, false, 0, 0);
    assertEquals(
        Output.stopped(), sequence.update(0.04, false, true, false, true, true, feedback(0, 0)));
    assertEquals(Output.stopped(), tick(sequence, 0.06, true, true, 0, 0));
    assertEquals(State.WAIT_FOR_RELEASE, sequence.state());
  }

  @Test
  void simulationCompletesIntakeFeedAndBoundedAgitation() {
    Settings agitation = settings.withAgitation(true);
    var sim = new FuelIOSim(agitation, false);
    var sequence = new FuelSequencer(agitation);
    var inputs = new FuelIO.FuelIOInputs();
    var visited = EnumSet.noneOf(State.class);
    for (int frame = 0; frame < 1000; frame++) {
      sim.updateInputs(inputs);
      boolean intake = frame > 1 && frame < 100;
      boolean shoot = frame >= 130;
      var output =
          sequence.update(
              frame * 0.02,
              true,
              true,
              false,
              intake,
              shoot,
              new Feedback(
                  inputs.available,
                  inputs.healthy,
                  inputs.referenced,
                  inputs.pivotDegrees,
                  inputs.shooterPrimaryMotorRpm,
                  inputs.shooterSecondaryMotorRpm,
                  inputs.ageSeconds));
      sim.apply(output);
      visited.add(sequence.state());
      assertNotEquals(State.FAULT, sequence.state(), sequence.fault());
      if (output.pivotEnabled()) {
        assertTrue(output.pivotDegrees() >= settings.minDegrees());
        assertTrue(output.pivotDegrees() <= settings.maxDegrees());
      }
    }
    assertTrue(
        visited.containsAll(
            EnumSet.of(
                State.DEPLOYING,
                State.INTAKING,
                State.DEPLOYED,
                State.SPINNING_UP,
                State.FEEDING,
                State.AGITATING_DOWN,
                State.AGITATING_UP)));
    assertEquals(State.FEEDING, sequence.state());
    assertEquals(settings.feedDegrees(), inputs.pivotDegrees, settings.positionToleranceDegrees());
  }

  @Test
  void defaultSequenceNeverAgitatesOrClaimsHopperIsEmpty() {
    var sequence = armed();
    tick(sequence, 0.02, false, true, 0, 3000);
    tick(sequence, 0.23, false, true, 0, 3000);
    var output = tick(sequence, 10, false, true, 70, 3000);
    assertEquals(State.FEEDING, sequence.state());
    assertEquals(settings.shooterPrimaryMotorRpm(), output.shooterPrimaryMotorRpm());
    assertEquals(settings.feedDegrees(), output.pivotDegrees());
  }

  @org.junit.jupiter.params.ParameterizedTest
  @org.junit.jupiter.params.provider.ValueSource(booleans = {true, false})
  void eitherShooterMustBeReadyAndIndexerStopsOnEitherSpeedDip(boolean primarySlow) {
    var sequence = armed();
    var oneSlow =
        new Feedback(true, true, true, 0, primarySlow ? 1000 : 3000, primarySlow ? 3000 : 1000, 0);
    for (int frame = 1; frame <= 30; frame++) {
      var output = sequence.update(frame * .02, true, true, false, false, true, oneSlow);
      assertEquals(State.SPINNING_UP, sequence.state());
      assertEquals(0, output.indexerDuty());
      assertEquals(0, output.pivotDegrees());
      assertEquals(3000, output.shooterPrimaryMotorRpm());
      assertEquals(3000, output.shooterSecondaryMotorRpm());
    }
    tick(sequence, .62, false, true, 0, 3000);
    var feeding = tick(sequence, .84, false, true, 0, 3000);
    assertEquals(settings.indexerDutyCycle(), feeding.indexerDuty());
    var paused =
        sequence.update(
            .86,
            true,
            true,
            false,
            false,
            true,
            new Feedback(
                true, true, true, 20, primarySlow ? 2000 : 3000, primarySlow ? 3000 : 2000, 0));
    assertEquals(0, paused.indexerDuty());
    assertEquals(20, paused.pivotDegrees());
    tick(sequence, .88, false, true, 20, 3000);
    assertEquals(0, tick(sequence, .98, false, true, 20, 3000).indexerDuty());
    assertTrue(tick(sequence, 1.1, false, true, 20, 3000).indexerDuty() > 0);
    var stopped = tick(sequence, 1.12, false, false, 25, 3000);
    assertEquals(0, stopped.indexerDuty());
    assertEquals(0, stopped.shooterPrimaryMotorRpm());
    assertEquals(0, stopped.shooterSecondaryMotorRpm());
  }

  @Test
  void averageSpeedCannotHideMismatchedMotorsAndInvalidSecondaryFeedbackFaults() {
    var sequence = armed();
    var mismatch = new Feedback(true, true, true, 0, 2800, 3200, 0);
    sequence.update(.02, true, true, false, false, true, mismatch);
    var output = sequence.update(.5, true, true, false, false, true, mismatch);
    assertEquals(0, output.indexerDuty());
    assertEquals(State.SPINNING_UP, sequence.state());
    output =
        sequence.update(
            .52,
            true,
            true,
            false,
            false,
            true,
            new Feedback(true, true, true, 0, 3000, Double.NaN, 0));
    assertEquals(Output.stopped(), output);
    assertEquals(State.FAULT, sequence.state());
  }

  @Test
  void indexerNeverRunsDuringIntakeAndBothShootersStopOnTimeout() {
    var sequence = armed();
    assertEquals(0, tick(sequence, .02, true, false, 0, 0).indexerDuty());
    tick(sequence, .04, false, true, 0, 0);
    assertEquals(Output.stopped(), tick(sequence, 4.1, false, true, 0, 0));
  }

  @Test
  void invalidSettingsFailBeforeAnyHardwareIsCreated() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            new Settings(
                0, 80, 0, 70, 50, 2, Double.NaN, 150, .2, 4, 3, .6, .15, 3, false, .1, .5, .35));
    assertThrows(
        IllegalArgumentException.class,
        () ->
            new Settings(0, 80, 0, 70, 90, 2, 3000, 150, .2, 4, 3, .6, .15, 3, true, .1, .5, .35));
  }

  @Test
  void independentMotorTargetsAndTolerancesAreNotAveraged() {
    var s =
        new Settings(
            0, 80, 70, 5, 25, 2, 3000, 100, .2, 4, 3, .6, .15, 3, false, .1, .5, .35, 4000, 50);
    var sequence = new FuelSequencer(s);
    sequence.update(
        0, true, true, false, false, false, new Feedback(true, true, true, 70, 0, 0, 0));
    var output =
        sequence.update(
            .02, true, true, false, false, true, new Feedback(true, true, true, 70, 3000, 3900, 0));
    assertEquals(3000, output.shooterPrimaryMotorRpm());
    assertEquals(4000, output.shooterSecondaryMotorRpm());
    output =
        sequence.update(
            .5, true, true, false, false, true, new Feedback(true, true, true, 70, 3000, 3900, 0));
    assertEquals(0, output.indexerDuty());
    sequence.update(
        .52, true, true, false, false, true, new Feedback(true, true, true, 70, 3000, 4000, 0));
    output =
        sequence.update(
            .74, true, true, false, false, true, new Feedback(true, true, true, 70, 3000, 4000, 0));
    assertTrue(output.indexerDuty() > 0);
  }

  @Test
  void unusedAgitationTimingCanRemainUnset() {
    assertDoesNotThrow(
        () -> new Settings(0, 80, 70, 5, 0, 2, 3000, 100, .2, 4, 3, 0, 0, 0, false, .1, .5, .35));
  }
}
