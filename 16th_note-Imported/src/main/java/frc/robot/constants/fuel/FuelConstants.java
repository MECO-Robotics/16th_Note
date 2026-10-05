package frc.robot.constants.fuel;

/** Sequence settings are mechanism units, never motor rotations. */
public final class FuelConstants {
  public static final FuelConfiguration REAL = FuelConfiguration.unconfigured();

  // Reserved IDs only; this does not configure or instantiate any hardware.
  public static final int SHOOTER_PRIMARY_CAN_ID = 3;
  public static final int INTAKE_PIVOT_CAN_ID = 4;
  public static final int INTAKE_ROLLER_CAN_ID = 5;
  public static final int SHOOTER_SECONDARY_CAN_ID = 6;
  public static final int INDEXER_CAN_ID = 7;

  /** Example values for software simulation only; none are calibrated robot measurements. */
  public static final Settings SIMULATION =
      new Settings(0, 80, 70, 5, 25, 2, 3000, 150, 0.2, 4, 3, 0.6, 0.15, 3, false, 0.1, 0.5, 0.35);

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
