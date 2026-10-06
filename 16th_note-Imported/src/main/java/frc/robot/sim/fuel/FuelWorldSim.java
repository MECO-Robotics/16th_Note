package frc.robot.sim.fuel;

import edu.wpi.first.math.geometry.*;
import java.util.ArrayList;
import java.util.List;

/** Practice-only ball model. No collisions, drag, active-hub rules, or calibrated shot physics. */
public final class FuelWorldSim {
  public static final double RADIUS = .075;
  // Approximate 2026 hub locations in blue-origin coordinates; scoring aperture is an estimate.
  public static final double HUB_X = 4.625467, HUB_Y = 4.034663, RED_HUB_X = 11.915521;
  public static final double HUB_HEIGHT = 1.8288, APERTURE_RADIUS = .45;
  public static final Pose2d START = new Pose2d(HUB_X - 1.35, HUB_Y, new Rotation2d());
  public static final double HOOD_ANGLE_DEGREES = 80;
  private static final double MUZZLE_HEIGHT = .65;
  /** Maximum height of the TOP of a simulated fuel ball above carpet. */
  public static final double DEFAULT_PEAK_LIMIT_METERS = 9 * .3048;

  public static final double DEFAULT_LAUNCH_SPEED_MPS =
      Math.sqrt(2 * 9.81 * (DEFAULT_PEAK_LIMIT_METERS - RADIUS - MUZZLE_HEIGHT))
          / Math.sin(Math.toRadians(HOOD_ANGLE_DEGREES));

  public static double predictedPeakMeters(double motorRpm, double launchSpeed) {
    double speed = launchSpeed * Math.min(motorRpm / 3000.0, 2);
    double vertical = speed * Math.sin(Math.toRadians(HOOD_ANGLE_DEGREES));
    return MUZZLE_HEIGHT + RADIUS + vertical * vertical / (2 * 9.81);
  }

  private String shotBlockedReason = "";

  public String shotBlockedReason() {
    return shotBlockedReason;
  }
  // A static practice placement for the example ballistic parameters, not automatic aiming.
  private static double defaultShotRange() {
    double pitch = Math.toRadians(HOOD_ANGLE_DEGREES);
    double vertical = DEFAULT_LAUNCH_SPEED_MPS * Math.sin(pitch);
    double time =
        (vertical + Math.sqrt(vertical * vertical - 2 * 9.81 * (HUB_HEIGHT - MUZZLE_HEIGHT)))
            / 9.81;
    return DEFAULT_LAUNCH_SPEED_MPS * Math.cos(pitch) * time;
  }
  /** Rear shooter (-X) faces the blue hub; intake remains on robot +X. */
  public static final Pose2d SHOOTING_START =
      new Pose2d(HUB_X - .15 - defaultShotRange(), HUB_Y, Rotation2d.fromDegrees(180));

  public static final int NEUTRAL_FUEL_COUNT = 400;
  public static final double FUEL_SPACING = .16;
  public static final double FIELD_CENTER_X = (HUB_X + RED_HUB_X) / 2;
  public static final double FUEL_MIN_X = FIELD_CENTER_X - 9.5 * FUEL_SPACING;
  public static final double FUEL_MIN_Y = HUB_Y - 9.5 * FUEL_SPACING;
  public static final Pose2d PICKUP_START =
      new Pose2d(FUEL_MIN_X - .65, FUEL_MIN_Y, new Rotation2d());
  public static final double PICKUP_RADIUS = .35;
  private Translation2d previousMouth;
  private final List<Translation3d> ground = new ArrayList<>();
  private final List<Shot> shots = new ArrayList<>();
  private int held, scored, launched;
  private double cooldown;

  private static final class Shot {
    Translation3d position;
    final double vx, vy;
    double vz;

    Shot(Translation3d position, double vx, double vy, double vz) {
      this.position = position;
      this.vx = vx;
      this.vy = vy;
      this.vz = vz;
    }
  }

  public FuelWorldSim() {
    reset(0);
  }

  public void reset(int preloads) {
    ground.clear();
    shots.clear();
    held = Math.max(0, Math.min(6, preloads));
    scored = 0;
    launched = 0;
    cooldown = 0;
    previousMouth = null;
    for (int i = 0; i < NEUTRAL_FUEL_COUNT; i++)
      ground.add(
          new Translation3d(
              FUEL_MIN_X + (i % 20) * FUEL_SPACING, FUEL_MIN_Y + (i / 20) * FUEL_SPACING, RADIUS));
  }
  /** Used for deterministic exercises and tests; positions are field-relative meters. */
  public void addFuel(Translation2d position) {
    ground.add(new Translation3d(position.getX(), position.getY(), RADIUS));
  }

  public void update(
      double dt,
      Pose2d robot,
      boolean enabled,
      boolean intake,
      boolean feed,
      double motorRpm,
      double launchSpeed) {
    update(dt, robot, enabled, intake, feed, motorRpm, launchSpeed, DEFAULT_PEAK_LIMIT_METERS);
  }

  public void update(
      double dt,
      Pose2d robot,
      boolean enabled,
      boolean intake,
      boolean feed,
      double motorRpm,
      double launchSpeed,
      double peakLimitMeters) {
    if (!Double.isFinite(dt) || dt <= 0 || dt > .1) return;
    shotBlockedReason =
        !Double.isFinite(peakLimitMeters) || peakLimitMeters <= MUZZLE_HEIGHT + RADIUS
            ? "Invalid simulated peak-height limit"
            : !Double.isFinite(motorRpm)
                    || motorRpm <= 0
                    || !Double.isFinite(launchSpeed)
                    || launchSpeed <= 0
                    || launchSpeed > 15
                ? "Invalid or stopped simulated launch speed"
                : predictedPeakMeters(motorRpm, launchSpeed) > peakLimitMeters + 1e-9
                    ? "Simulated shot exceeds peak-height limit"
                    : "";
    cooldown = Math.max(0, cooldown - dt);
    if (enabled && intake) {
      var mouth = pickupMouth(robot);
      // Sweep only contiguous active pickup motion; never collect along a teleport or reset.
      var from =
          previousMouth != null && previousMouth.getDistance(mouth) < 2 ? previousMouth : mouth;
      for (var it = ground.iterator(); it.hasNext() && held < 6; ) {
        if (segmentDistance(it.next().toTranslation2d(), from, mouth) <= PICKUP_RADIUS) {
          it.remove();
          held++;
        }
      }
      previousMouth = mouth;
    } else previousMouth = null;
    if (enabled
        && feed
        && shotBlockedReason.isEmpty()
        && held > 0
        && cooldown == 0
        && Double.isFinite(motorRpm)
        && motorRpm > 0
        && Double.isFinite(launchSpeed)
        && launchSpeed > 0
        && launchSpeed <= 15) {
      double yaw = robot.getRotation().getRadians() + Math.PI,
          pitch = Math.toRadians(HOOD_ANGLE_DEGREES);
      double speed = launchSpeed * Math.min(motorRpm / 3000.0, 2);
      var origin =
          robot.getTranslation().plus(new Translation2d(-.15, 0).rotateBy(robot.getRotation()));
      shots.add(
          new Shot(
              new Translation3d(origin.getX(), origin.getY(), MUZZLE_HEIGHT),
              speed * Math.cos(pitch) * Math.cos(yaw),
              speed * Math.cos(pitch) * Math.sin(yaw),
              speed * Math.sin(pitch)));
      held--;
      launched++;
      cooldown = .3;
    }
    // Existing projectiles keep falling after disable; disable prevents new shots and pickups.
    for (var it = shots.iterator(); it.hasNext(); ) {
      var shot = it.next();
      var previous = shot.position;
      var next =
          previous.plus(
              new Translation3d(shot.vx * dt, shot.vy * dt, shot.vz * dt - .5 * 9.81 * dt * dt));
      shot.vz -= 9.81 * dt;
      if (crossesHub(previous, next)) {
        scored++;
        it.remove();
      } else if (next.getZ() <= RADIUS) {
        if (next.getX() >= 0 && next.getX() <= 16.55 && next.getY() >= 0 && next.getY() <= 8.07)
          ground.add(new Translation3d(next.getX(), next.getY(), RADIUS));
        it.remove();
      } else shot.position = next;
    }
  }

  public static Translation2d pickupMouth(Pose2d robot) {
    return robot.getTranslation().plus(new Translation2d(.32, 0).rotateBy(robot.getRotation()));
  }

  private static double segmentDistance(Translation2d point, Translation2d a, Translation2d b) {
    var delta = b.minus(a);
    double lengthSquared = delta.getX() * delta.getX() + delta.getY() * delta.getY();
    if (lengthSquared < 1e-12) return point.getDistance(b);
    var relative = point.minus(a);
    double t =
        edu.wpi.first.math.MathUtil.clamp(
            (relative.getX() * delta.getX() + relative.getY() * delta.getY()) / lengthSquared,
            0,
            1);
    return point.getDistance(a.plus(delta.times(t)));
  }

  public double nearestFuelDistance(Pose2d robot) {
    var mouth = pickupMouth(robot);
    return ground.stream()
        .mapToDouble(p -> p.toTranslation2d().getDistance(mouth))
        .min()
        .orElse(-1);
  }

  public int groundCount() {
    return ground.size();
  }

  private static boolean crossesHub(Translation3d a, Translation3d b) {
    if (a.getZ() <= HUB_HEIGHT || b.getZ() > HUB_HEIGHT) return false;
    double t = (a.getZ() - HUB_HEIGHT) / (a.getZ() - b.getZ());
    double x = a.getX() + t * (b.getX() - a.getX());
    double y = a.getY() + t * (b.getY() - a.getY());
    return Math.hypot(x - HUB_X, y - HUB_Y) < APERTURE_RADIUS - RADIUS
        || Math.hypot(x - RED_HUB_X, y - HUB_Y) < APERTURE_RADIUS - RADIUS;
  }

  public int held() {
    return held;
  }

  public int scored() {
    return scored;
  }

  public int launched() {
    return launched;
  }

  public Pose3d[] groundPoses() {
    return ground.stream().map(p -> new Pose3d(p, new Rotation3d())).toArray(Pose3d[]::new);
  }

  public Pose3d[] projectilePoses() {
    return shots.stream().map(s -> new Pose3d(s.position, new Rotation3d())).toArray(Pose3d[]::new);
  }

  public Pose3d[] heldPoses(Pose2d robot) {
    var poses = new Pose3d[held];
    for (int i = 0; i < held; i++)
      poses[i] =
          new Pose3d(robot)
              .transformBy(
                  new Transform3d(
                      new Translation3d(-.1 + (i % 2) * .16, -.16 + (i / 2) * .16, .35),
                      new Rotation3d()));
    return poses;
  }
}
