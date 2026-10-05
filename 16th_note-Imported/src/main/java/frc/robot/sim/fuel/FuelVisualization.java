package frc.robot.sim.fuel;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.constants.fuel.FuelConstants;

/** CAD visualization only; these offsets never configure real motor control or travel limits. */
public final class FuelVisualization {
  // Shared jackshaft/indexer axis from the October 5 STEP exports, in robot coordinates.
  public static final double PIVOT_X_METERS = 0.0965173097561094;
  public static final double PIVOT_Z_METERS = 0.3551707016573203;
  // The export is intake-down; the user confirmed 160 degrees from stowed to deployed.

  public static Pose3d[] componentPoses(double intakeDegrees) {
    return new Pose3d[] {
      new Pose3d(
          PIVOT_X_METERS,
          0,
          PIVOT_Z_METERS,
          new Rotation3d(
              0, Math.toRadians(intakeDegrees - FuelConstants.INTAKE_DEPLOYED_DEGREES), 0))
    };
  }

  private FuelVisualization() {}
}
