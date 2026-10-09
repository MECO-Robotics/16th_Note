package frc.robot.constants.fuel;

/** Pivot settings use intake degrees; shooter targets use motor RPM. */
public final class FuelConstants {
  /**
   * Provisional speed starting point from 2026-Rebuilt's close-hub preset: 30 wheel RPS * 60 *
   * 22/14 motor rotations per wheel rotation. The owner confirmed 14T motor pulleys driving 22T
   * flywheel pulleys on both sides of 16th Note. This is not a calibrated shot or a guarantee of
   * equal ball exit speed. Tune each motor separately.
   */
  public static final double INITIAL_SHOOTER_PRIMARY_MOTOR_RPM = 30.0 * 60.0 * (22.0 / 14.0);

  public static final double INITIAL_SHOOTER_SECONDARY_MOTOR_RPM =
      INITIAL_SHOOTER_PRIMARY_MOTOR_RPM;

  /** Confirmed stowed-to-deployed intake travel; upper/stowed reference is zero. */
  public static final double INTAKE_DEPLOYED_DEGREES = 160;

  public static final FuelConfiguration REAL = FuelConfiguration.unconfigured();

  // Reserved IDs only; this does not configure or instantiate any hardware.
  public static final int SHOOTER_PRIMARY_CAN_ID = 3;
  public static final int SHOOTER_SECONDARY_CAN_ID = 4;
  public static final int INDEXER_PRIMARY_CAN_ID = 5;
  public static final int INDEXER_SECONDARY_CAN_ID = 6;
  public static final int INTAKE_PIVOT_CAN_ID = 7;

  /** Example values for software simulation only; none are calibrated robot measurements. */
  public static final Settings SIMULATION =
      new Settings(
          0,
          INTAKE_DEPLOYED_DEGREES,
          INTAKE_DEPLOYED_DEGREES,
          5,
          25,
          2,
          3000,
          150,
          0.2,
          4,
          3,
          0.6,
          0.15,
          3,
          false,
          0.1,
          0.5,
          0.35);

  public record Settings(
      double minDegrees,
      double maxDegrees,
      double intakeDegrees,
      double feedDegrees,
      double agitationDegrees,
      double positionToleranceDegrees,
      double shooterPrimaryMotorRpm,
      double primarySpeedToleranceMotorRpm,
      double readySeconds,
      double spinupTimeoutSeconds,
      double motionTimeoutSeconds,
      double feedDwellSeconds,
      double agitationDwellSeconds,
      int maxAgitationCycles,
      boolean agitationEnabled,
      double maxFeedbackAgeSeconds,
      double intakeDutyCycle,
      double indexerDutyCycle,
      double shooterSecondaryMotorRpm,
      double secondarySpeedToleranceMotorRpm) {
    public Settings {
      double[] values = {
        minDegrees,
        maxDegrees,
        intakeDegrees,
        feedDegrees,
        agitationDegrees,
        positionToleranceDegrees,
        shooterPrimaryMotorRpm,
        primarySpeedToleranceMotorRpm,
        readySeconds,
        spinupTimeoutSeconds,
        motionTimeoutSeconds,
        feedDwellSeconds,
        agitationDwellSeconds,
        maxFeedbackAgeSeconds,
        intakeDutyCycle,
        indexerDutyCycle,
        shooterSecondaryMotorRpm,
        secondarySpeedToleranceMotorRpm
      };
      for (double value : values) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException("Nonfinite fuel setting");
      }
      if (minDegrees >= maxDegrees
          || intakeDegrees < minDegrees
          || intakeDegrees > maxDegrees
          || feedDegrees < minDegrees
          || feedDegrees > maxDegrees
          || agitationDegrees < minDegrees
          || agitationDegrees > maxDegrees
          || positionToleranceDegrees <= 0
          || Math.abs(feedDegrees - intakeDegrees) <= 2 * positionToleranceDegrees
          || shooterPrimaryMotorRpm <= 0
          || primarySpeedToleranceMotorRpm <= 0
          || primarySpeedToleranceMotorRpm >= shooterPrimaryMotorRpm
          || readySeconds <= 0
          || spinupTimeoutSeconds <= readySeconds
          || motionTimeoutSeconds <= 0
          || feedDwellSeconds < 0
          || agitationDwellSeconds < 0
          || maxAgitationCycles < 0
          || maxFeedbackAgeSeconds <= 0
          || intakeDutyCycle <= 0
          || intakeDutyCycle > 1
          || indexerDutyCycle <= 0
          || indexerDutyCycle > 1
          || shooterSecondaryMotorRpm <= 0
          || secondarySpeedToleranceMotorRpm <= 0
          || secondarySpeedToleranceMotorRpm >= shooterSecondaryMotorRpm) {
        throw new IllegalArgumentException("Invalid fuel sequence settings");
      }
      if (agitationEnabled
          && (maxAgitationCycles == 0
              || feedDwellSeconds <= 0
              || agitationDwellSeconds <= 0
              || agitationDegrees <= Math.min(intakeDegrees, feedDegrees) + positionToleranceDegrees
              || agitationDegrees
                  >= Math.max(intakeDegrees, feedDegrees) - positionToleranceDegrees)) {
        throw new IllegalArgumentException("Agitation target must be inside intake/feed travel");
      }
    }

    public Settings(
        double minDegrees,
        double maxDegrees,
        double intakeDegrees,
        double feedDegrees,
        double agitationDegrees,
        double positionToleranceDegrees,
        double shooterPrimaryMotorRpm,
        double primarySpeedToleranceMotorRpm,
        double readySeconds,
        double spinupTimeoutSeconds,
        double motionTimeoutSeconds,
        double feedDwellSeconds,
        double agitationDwellSeconds,
        int maxAgitationCycles,
        boolean agitationEnabled,
        double maxFeedbackAgeSeconds,
        double intakeDutyCycle,
        double indexerDutyCycle) {
      this(
          minDegrees,
          maxDegrees,
          intakeDegrees,
          feedDegrees,
          agitationDegrees,
          positionToleranceDegrees,
          shooterPrimaryMotorRpm,
          primarySpeedToleranceMotorRpm,
          readySeconds,
          spinupTimeoutSeconds,
          motionTimeoutSeconds,
          feedDwellSeconds,
          agitationDwellSeconds,
          maxAgitationCycles,
          agitationEnabled,
          maxFeedbackAgeSeconds,
          intakeDutyCycle,
          indexerDutyCycle,
          shooterPrimaryMotorRpm,
          primarySpeedToleranceMotorRpm);
    }

    public Settings withShooterMotorRpm(double primary, double secondary) {
      return new Settings(
          minDegrees,
          maxDegrees,
          intakeDegrees,
          feedDegrees,
          agitationDegrees,
          positionToleranceDegrees,
          primary,
          primarySpeedToleranceMotorRpm,
          readySeconds,
          spinupTimeoutSeconds,
          motionTimeoutSeconds,
          feedDwellSeconds,
          agitationDwellSeconds,
          maxAgitationCycles,
          agitationEnabled,
          maxFeedbackAgeSeconds,
          intakeDutyCycle,
          indexerDutyCycle,
          secondary,
          secondarySpeedToleranceMotorRpm);
    }

    public Settings withIntakeAngles(double feed, double agitation) {
      return new Settings(
          minDegrees,
          maxDegrees,
          intakeDegrees,
          feed,
          agitation,
          positionToleranceDegrees,
          shooterPrimaryMotorRpm,
          primarySpeedToleranceMotorRpm,
          readySeconds,
          spinupTimeoutSeconds,
          motionTimeoutSeconds,
          feedDwellSeconds,
          agitationDwellSeconds,
          maxAgitationCycles,
          agitationEnabled,
          maxFeedbackAgeSeconds,
          intakeDutyCycle,
          indexerDutyCycle,
          shooterSecondaryMotorRpm,
          secondarySpeedToleranceMotorRpm);
    }

    public Settings withAgitation(boolean enabled) {
      return new Settings(
          minDegrees,
          maxDegrees,
          intakeDegrees,
          feedDegrees,
          agitationDegrees,
          positionToleranceDegrees,
          shooterPrimaryMotorRpm,
          primarySpeedToleranceMotorRpm,
          readySeconds,
          spinupTimeoutSeconds,
          motionTimeoutSeconds,
          feedDwellSeconds,
          agitationDwellSeconds,
          maxAgitationCycles,
          enabled,
          maxFeedbackAgeSeconds,
          intakeDutyCycle,
          indexerDutyCycle,
          shooterSecondaryMotorRpm,
          secondarySpeedToleranceMotorRpm);
    }
  }

  private FuelConstants() {}
}
