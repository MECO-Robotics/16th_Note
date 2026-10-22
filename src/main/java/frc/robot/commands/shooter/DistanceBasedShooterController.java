package frc.robot.commands.shooter;

/** Piecewise-linear flywheel calibration over three measured distance/speed points. */
public final class DistanceBasedShooterController {
  public record Output(boolean configured, boolean inRange, double targetRps) {}

  private DistanceBasedShooterController() {}

  public static Output calculate(
      double distanceMeters,
      double closeDistanceMeters,
      double closeRps,
      double middleDistanceMeters,
      double middleRps,
      double farDistanceMeters,
      double farRps,
      double maximumRps) {
    if (!allFinite(
            closeDistanceMeters,
            closeRps,
            middleDistanceMeters,
            middleRps,
            farDistanceMeters,
            farRps,
            maximumRps)
        || closeDistanceMeters <= 0.0
        || closeRps <= 0.0
        || closeRps > maximumRps
        || middleDistanceMeters <= closeDistanceMeters
        || middleRps <= 0.0
        || middleRps > maximumRps
        || farDistanceMeters <= middleDistanceMeters
        || farRps <= 0.0
        || farRps > maximumRps
        || maximumRps <= 0.0) {
      return new Output(false, false, 0.0);
    }

    if (!Double.isFinite(distanceMeters)
        || distanceMeters < closeDistanceMeters
        || distanceMeters > farDistanceMeters) {
      return new Output(true, false, 0.0);
    }

    double targetRps =
        distanceMeters <= middleDistanceMeters
            ? interpolate(
                distanceMeters, closeDistanceMeters, closeRps, middleDistanceMeters, middleRps)
            : interpolate(
                distanceMeters, middleDistanceMeters, middleRps, farDistanceMeters, farRps);
    return new Output(true, true, targetRps);
  }

  private static double interpolate(double x, double x0, double y0, double x1, double y1) {
    return y0 + (x - x0) / (x1 - x0) * (y1 - y0);
  }

  private static boolean allFinite(double... values) {
    for (double value : values) {
      if (!Double.isFinite(value)) return false;
    }
    return true;
  }
}
