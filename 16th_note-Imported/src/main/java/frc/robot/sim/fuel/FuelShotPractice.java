package frc.robot.sim.fuel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Fixed blue-alliance practice locations, not measured/calibrated hardware shots or auto aiming.
 */
public final class FuelShotPractice {
  public record Preset(String name, Pose2d pose, double motorRpm, double peakLimitFeet) {}

  public static final Preset CLOSE = new Preset("Close", FuelWorldSim.SHOOTING_START, 3000, 9);
  // Clear of the trench on the alliance side; left/right as viewed from blue driver stations.
  public static final Preset LEFT = trench("Left trench", 8.07 - .8);
  public static final Preset RIGHT = trench("Right trench", .8);

  private static Preset trench(String name, double y) {
    double x = FuelWorldSim.HUB_X - .95;
    double dx = FuelWorldSim.HUB_X - x, dy = FuelWorldSim.HUB_Y - y;
    var pose = new Pose2d(x, y, new Rotation2d(Math.atan2(dy, dx) + Math.PI));
    // Rear muzzle sits 0.15 m toward the hub from the robot center.
    double distance = Math.hypot(dx, dy) - .15;
    double angle = Math.toRadians(FuelWorldSim.HOOD_ANGLE_DEGREES);
    double speed =
        Math.sqrt(
            9.81
                * distance
                * distance
                / (2
                    * Math.pow(Math.cos(angle), 2)
                    * (distance * Math.tan(angle) - (FuelWorldSim.HUB_HEIGHT - .65))));
    return new Preset(name, pose, 3000 * speed / FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS, 19);
  }

  private FuelShotPractice() {}
}
