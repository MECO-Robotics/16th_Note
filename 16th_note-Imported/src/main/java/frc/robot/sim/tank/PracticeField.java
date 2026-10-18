package frc.robot.sim.tank;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

/** Approximate flat practice boundaries; not a full field physics engine. */
public final class PracticeField {
  public static final double LENGTH = 16.55, WIDTH = 8.07;
  public static final double HUB_X = 4.625467, HUB_Y = 4.034663, RED_HUB_X = 11.915521;
  public static final Pose2d START =
      new Pose2d(HUB_X - 1.35, HUB_Y, new edu.wpi.first.math.geometry.Rotation2d());
  // Conservative circular chassis envelope and square hub footprint, in meters.
  public static final double ROBOT_RADIUS = .32, HUB_HALF_WIDTH = .60;

  private PracticeField() {}

  public static boolean blocked(double x, double y) {
    if (x < ROBOT_RADIUS
        || x > LENGTH - ROBOT_RADIUS
        || y < ROBOT_RADIUS
        || y > WIDTH - ROBOT_RADIUS) return true;
    for (double hubX : new double[] {PracticeField.HUB_X, PracticeField.RED_HUB_X}) {
      double nearestX = MathUtil.clamp(x, hubX - HUB_HALF_WIDTH, hubX + HUB_HALF_WIDTH);
      double nearestY =
          MathUtil.clamp(
              y, PracticeField.HUB_Y - HUB_HALF_WIDTH, PracticeField.HUB_Y + HUB_HALF_WIDTH);
      if (Math.hypot(x - nearestX, y - nearestY) < ROBOT_RADIUS) return true;
    }
    return false;
  }

  /** Sweep the motion so a large step cannot pass through a hub or field wall. */
  public static Pose2d constrain(Pose2d previous, Pose2d commanded) {
    int steps =
        Math.max(
            1,
            (int)
                Math.ceil(previous.getTranslation().getDistance(commanded.getTranslation()) / .02));
    double x = previous.getX(), y = previous.getY();
    for (int i = 1; i <= steps; i++) {
      double t = (double) i / steps;
      double nextX = previous.getX() + (commanded.getX() - previous.getX()) * t;
      double nextY = previous.getY() + (commanded.getY() - previous.getY()) * t;
      if (blocked(nextX, nextY)) break;
      x = nextX;
      y = nextY;
    }
    return new Pose2d(x, y, commanded.getRotation());
  }
}
