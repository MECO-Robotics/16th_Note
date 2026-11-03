package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Preferences;

/** Loads and saves the measured distance-shooter table in roboRIO persistent storage. */
public final class DistanceShooterCalibrationStore {
  private static final String CLOSE_DISTANCE_KEY = "DistanceShooter.CloseDistanceMeters";
  private static final String CLOSE_RPS_KEY = "DistanceShooter.CloseRps";
  private static final String MIDDLE_DISTANCE_KEY = "DistanceShooter.MiddleDistanceMeters";
  private static final String MIDDLE_RPS_KEY = "DistanceShooter.MiddleRps";
  private static final String FAR_DISTANCE_KEY = "DistanceShooter.FarDistanceMeters";
  private static final String FAR_RPS_KEY = "DistanceShooter.FarRps";

  public record Calibration(
      double closeDistanceMeters,
      double closeRps,
      double middleDistanceMeters,
      double middleRps,
      double farDistanceMeters,
      double farRps) {
    public boolean isValid(double maximumRps) {
      return DistanceBasedShooterController.calculate(
              middleDistanceMeters,
              closeDistanceMeters,
              closeRps,
              middleDistanceMeters,
              middleRps,
              farDistanceMeters,
              farRps,
              maximumRps)
          .configured();
    }

    public static Calibration unconfigured() {
      return new Calibration(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
  }

  private DistanceShooterCalibrationStore() {}

  public static Calibration load(double maximumRps) {
    initializeKeys();
    Calibration calibration =
        new Calibration(
            Preferences.getDouble(CLOSE_DISTANCE_KEY, 0.0),
            Preferences.getDouble(CLOSE_RPS_KEY, 0.0),
            Preferences.getDouble(MIDDLE_DISTANCE_KEY, 0.0),
            Preferences.getDouble(MIDDLE_RPS_KEY, 0.0),
            Preferences.getDouble(FAR_DISTANCE_KEY, 0.0),
            Preferences.getDouble(FAR_RPS_KEY, 0.0));
    return calibration.isValid(maximumRps) ? calibration : Calibration.unconfigured();
  }

  public static boolean save(Calibration calibration, double maximumRps) {
    if (!calibration.isValid(maximumRps)) return false;
    Preferences.setDouble(CLOSE_DISTANCE_KEY, calibration.closeDistanceMeters());
    Preferences.setDouble(CLOSE_RPS_KEY, calibration.closeRps());
    Preferences.setDouble(MIDDLE_DISTANCE_KEY, calibration.middleDistanceMeters());
    Preferences.setDouble(MIDDLE_RPS_KEY, calibration.middleRps());
    Preferences.setDouble(FAR_DISTANCE_KEY, calibration.farDistanceMeters());
    Preferences.setDouble(FAR_RPS_KEY, calibration.farRps());
    return true;
  }

  private static void initializeKeys() {
    Preferences.initDouble(CLOSE_DISTANCE_KEY, 0.0);
    Preferences.initDouble(CLOSE_RPS_KEY, 0.0);
    Preferences.initDouble(MIDDLE_DISTANCE_KEY, 0.0);
    Preferences.initDouble(MIDDLE_RPS_KEY, 0.0);
    Preferences.initDouble(FAR_DISTANCE_KEY, 0.0);
    Preferences.initDouble(FAR_RPS_KEY, 0.0);
  }
}
