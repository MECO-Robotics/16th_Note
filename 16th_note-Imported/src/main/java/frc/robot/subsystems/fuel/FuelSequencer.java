package frc.robot.subsystems.fuel;

import frc.robot.constants.fuel.FuelConstants.Settings;

/** Deterministic mechanism coordination, independent of motor APIs and wall-clock timing. */
public final class FuelSequencer {
  public enum State {
    DISABLED,
    UNAVAILABLE,
    WAIT_FOR_RELEASE,
    IDLE,
    DEPLOYING,
    INTAKING,
    DEPLOYED,
    SPINNING_UP,
    FEEDING,
    AGITATING_DOWN,
    AGITATING_UP,
    FAULT
  }

  public record Feedback(
      boolean available,
      boolean healthy,
      boolean referenced,
      double pivotDegrees,
      double shooterPrimaryMotorRpm,
      double shooterSecondaryMotorRpm,
      double ageSeconds) {}

  /** A disabled pivot means neutral output, not a position command to zero. */
  public record Output(
      boolean pivotEnabled,
      double pivotDegrees,
      double shooterPrimaryMotorRpm,
      double shooterSecondaryMotorRpm,
      double intakeDuty,
      double indexerDuty) {
    public static Output stopped() {
      return new Output(false, 0, 0, 0, 0, 0);
    }
  }

  private final Settings settings;
  private State state = State.DISABLED;
  private String fault = "";
  private boolean armed;
  private boolean previousIntake;
  private boolean pivotEnabled;
  private double pivotGoal;
  private double motionStarted;
  private double stateStarted;
  private double readySince = Double.NaN;
  private double arrivedSince = Double.NaN;
  private int agitationCycles;

  public FuelSequencer(Settings settings) {
    this.settings = settings;
  }

  /** Only the autonomous owner may bypass human button-release arming. */
  public void armAutonomous(double position, double now) {
    if (fault.isEmpty()) {
      armed = true;
      state = State.IDLE;
      hold(position, now);
    }
  }

  public State state() {
    return state;
  }

  public String fault() {
    return fault;
  }

  public void stop() {
    state = State.DISABLED;
    armed = false;
    previousIntake = false;
    pivotEnabled = false;
    readySince = Double.NaN;
    arrivedSince = Double.NaN;
    // A fault is cleared by a disabled update, not by a command interruption.
  }

  public Output update(
      double now,
      boolean enabled,
      boolean connected,
      boolean brownedOut,
      boolean intake,
      boolean shoot,
      Feedback feedback) {
    if (!enabled) {
      stop();
      fault = "";
      return Output.stopped();
    }
    if (!fault.isEmpty()) return fail(fault);
    if (!feedback.available()) {
      stop();
      state = State.UNAVAILABLE;
      return Output.stopped();
    }
    if (!Double.isFinite(now)) return fail("Invalid clock");
    if (brownedOut) return fail("Brownout; disable before retrying");
    if (!connected) {
      stop();
      state = State.WAIT_FOR_RELEASE;
      return Output.stopped();
    }
    if (!feedback.healthy()
        || !Double.isFinite(feedback.ageSeconds())
        || feedback.ageSeconds() < 0
        || feedback.ageSeconds() > settings.maxFeedbackAgeSeconds()) {
      return fail("Disconnected or stale mechanism feedback");
    }
    if (!feedback.referenced()) return fail("Intake position is not referenced");
    if (!Double.isFinite(feedback.pivotDegrees())
        || !Double.isFinite(feedback.shooterPrimaryMotorRpm())
        || !Double.isFinite(feedback.shooterSecondaryMotorRpm())
        || feedback.pivotDegrees() < settings.minDegrees() - settings.positionToleranceDegrees()
        || feedback.pivotDegrees() > settings.maxDegrees() + settings.positionToleranceDegrees()) {
      return fail("Invalid mechanism measurement or intake outside travel");
    }
    if (!armed) {
      state = State.WAIT_FOR_RELEASE;
      if (!intake && !shoot) {
        armed = true;
        previousIntake = false;
        state = State.IDLE;
      }
      return Output.stopped();
    }

    boolean intakePressed = intake && !previousIntake;
    previousIntake = intake;
    if (shoot) {
      if (!isShooting()) {
        agitationCycles = 0;
        hold(feedback.pivotDegrees(), now);
        enter(State.SPINNING_UP, now);
      }
      boolean atSpeed =
          Math.abs(feedback.shooterPrimaryMotorRpm() - settings.shooterPrimaryMotorRpm())
                  <= settings.primarySpeedToleranceMotorRpm()
              && Math.abs(feedback.shooterSecondaryMotorRpm() - settings.shooterSecondaryMotorRpm())
                  <= settings.secondarySpeedToleranceMotorRpm();
      if (!atSpeed && state != State.SPINNING_UP) {
        hold(feedback.pivotDegrees(), now);
        enter(State.SPINNING_UP, now);
      }
      if (state == State.SPINNING_UP) {
        if (atSpeed) {
          if (Double.isNaN(readySince)) readySince = now;
          if (now - readySince >= settings.readySeconds()) {
            goal(settings.feedDegrees(), now);
            enter(State.FEEDING, now);
          }
        } else {
          readySince = Double.NaN;
        }
        if (state == State.SPINNING_UP && now - stateStarted >= settings.spinupTimeoutSeconds()) {
          return fail("Shooter did not reach stable speed before timeout");
        }
      }
      if (state != State.SPINNING_UP && atTarget(feedback)) {
        if (Double.isNaN(arrivedSince)) arrivedSince = now;
        if (settings.agitationEnabled()) {
          if (state == State.FEEDING
              && agitationCycles < settings.maxAgitationCycles()
              && now - arrivedSince >= settings.feedDwellSeconds()) {
            goal(settings.agitationDegrees(), now);
            enter(State.AGITATING_DOWN, now);
          } else if (state == State.AGITATING_DOWN
              && now - arrivedSince >= settings.agitationDwellSeconds()) {
            goal(settings.feedDegrees(), now);
            enter(State.AGITATING_UP, now);
          } else if (state == State.AGITATING_UP
              && now - arrivedSince >= settings.agitationDwellSeconds()) {
            agitationCycles++;
            enter(State.FEEDING, now);
          }
        }
      } else {
        arrivedSince = Double.NaN;
      }
    } else if (isShooting()) {
      // Releasing shoot ends feeding and returns to pickup position, even after agitation.
      // This is an operator request, not an inference that the hopper is empty.
      goal(settings.intakeDegrees(), now);
      enter(State.DEPLOYING, now);
    } else if (intakePressed) {
      goal(settings.intakeDegrees(), now);
      enter(State.DEPLOYING, now);
    }

    if (state == State.DEPLOYING || state == State.INTAKING || state == State.DEPLOYED) {
      state = atTarget(feedback) ? (intake ? State.INTAKING : State.DEPLOYED) : State.DEPLOYING;
    }
    if (pivotEnabled) {
      if (atTarget(feedback)) motionStarted = now;
      else if (now - motionStarted >= settings.motionTimeoutSeconds()) {
        return fail("Intake motion timed out; inspect for a jam or incorrect reference");
      }
    }
    return new Output(
        pivotEnabled,
        pivotGoal,
        isShooting() ? settings.shooterPrimaryMotorRpm() : 0,
        isShooting() ? settings.shooterSecondaryMotorRpm() : 0,
        state == State.INTAKING ? settings.intakeDutyCycle() : 0,
        isShooting() && state != State.SPINNING_UP ? settings.indexerDutyCycle() : 0);
  }

  private boolean isShooting() {
    return state == State.SPINNING_UP
        || state == State.FEEDING
        || state == State.AGITATING_DOWN
        || state == State.AGITATING_UP;
  }

  private boolean atTarget(Feedback feedback) {
    return Math.abs(feedback.pivotDegrees() - pivotGoal) <= settings.positionToleranceDegrees();
  }

  private void goal(double degrees, double now) {
    if (!pivotEnabled || pivotGoal != degrees) motionStarted = now;
    pivotGoal = degrees;
    pivotEnabled = true;
    arrivedSince = Double.NaN;
  }

  private void hold(double degrees, double now) {
    goal(Math.max(settings.minDegrees(), Math.min(settings.maxDegrees(), degrees)), now);
  }

  private void enter(State next, double now) {
    state = next;
    stateStarted = now;
    readySince = Double.NaN;
    arrivedSince = Double.NaN;
  }

  private Output fail(String reason) {
    fault = reason;
    state = State.FAULT;
    armed = false;
    pivotEnabled = false;
    return Output.stopped();
  }
}
