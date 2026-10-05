package frc.robot.constants.fuel;

import frc.robot.constants.fuel.FuelConstants.Settings;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Immutable configuration snapshots. Missing real values are never replaced by simulation values.
 */
public record FuelConfiguration(
    Map<Role, Device> devices,
    Coupling coupling,
    boolean followerCompatible,
    boolean followerInverted,
    Pivot pivot,
    Settings sequence,
    Auto auto) {
  public static final double PIVOT_RATIO = 20.0;
  public static final double DEGREES_PER_MOTOR_ROTATION = 360.0 / PIVOT_RATIO;

  public enum Role {
    SHOOTER_PRIMARY(3),
    PIVOT(4),
    ROLLERS(5),
    SHOOTER_SECONDARY(6),
    INDEXER(7);
    public final int canId;

    Role(int canId) {
      this.canId = canId;
    }
  }

  public enum Coupling {
    UNCONFIRMED,
    COUPLED_FOLLOWER,
    INDEPENDENT
  }

  public record Device(
      int currentAmps,
      boolean inverted,
      boolean directionConfirmed,
      int feedbackSign,
      double maxVolts,
      double testVolts,
      double kP,
      double kV) {
    public List<String> issues() {
      var issues = new ArrayList<String>();
      if (currentAmps <= 0 || currentAmps > 80) issues.add("current limit must be 1..80 A");
      if (!finitePositive(maxVolts) || maxVolts > 12) issues.add("max voltage must be >0 and <=12");
      if (!finitePositive(testVolts) || testVolts > maxVolts)
        issues.add("test voltage cap must be >0 and <=max voltage");
      if (feedbackSign != 1 && feedbackSign != -1) issues.add("feedback sign must be +1 or -1");
      if (!Double.isFinite(kP) || kP < 0 || !Double.isFinite(kV) || kV < 0)
        issues.add("gains must be finite and nonnegative");
      return issues;
    }

    public static Device missing() {
      return new Device(0, false, false, 1, 0, 0, 0, 0);
    }
  }

  public record Pivot(
      double kP,
      double kD,
      double gravityVolts,
      double maxDegreesPerSecond,
      double maxDegreesPerSecondSquared,
      double maxDegrees) {
    public boolean travelKnown() {
      return finitePositive(maxDegrees) && maxDegrees <= 180;
    }

    public boolean profileReady() {
      return finitePositive(kP)
          && Double.isFinite(kD)
          && kD >= 0
          && Double.isFinite(gravityVolts)
          && Math.abs(gravityVolts) <= 12
          && finitePositive(maxDegreesPerSecond)
          && finitePositive(maxDegreesPerSecondSquared);
    }

    public static Pivot missing() {
      return new Pivot(0, 0, 0, 0, 0, 0);
    }
  }

  public record Auto(
      boolean verified,
      double startDegrees,
      double leftVolts,
      double rightVolts,
      double backSeconds,
      double settleSeconds,
      double feedSeconds,
      double totalSeconds) {
    public List<String> issues() {
      var issues = new ArrayList<String>();
      if (!verified) issues.add("preload position/backing procedure not verified");
      if (!Double.isFinite(startDegrees)) issues.add("starting angle missing");
      if (!Double.isFinite(leftVolts)
          || !Double.isFinite(rightVolts)
          || leftVolts >= 0
          || rightVolts >= 0
          || leftVolts < -12
          || rightVolts < -12) issues.add("backing voltages must be negative and >=-12");
      if (!finitePositive(backSeconds)
          || !finitePositive(settleSeconds)
          || !finitePositive(feedSeconds)
          || !finitePositive(totalSeconds)
          || totalSeconds > 15
          || totalSeconds <= backSeconds + settleSeconds + feedSeconds)
        issues.add(
            "auto times must be positive with total <=15 s and greater than stage durations");
      return issues;
    }

    public static Auto missing() {
      return new Auto(false, Double.NaN, 0, 0, 0, 0, 0, 0);
    }
  }

  public FuelConfiguration {
    devices = Map.copyOf(devices);
  }

  public Device device(Role role) {
    return devices.getOrDefault(role, Device.missing());
  }

  public List<String> deviceIssues() {
    var result = new ArrayList<String>();
    for (Role role : Role.values())
      for (String issue : device(role).issues()) result.add(role + ": " + issue);
    return result;
  }

  public List<String> shooterIssues() {
    var result = new ArrayList<String>();
    if (coupling == Coupling.UNCONFIRMED) result.add("shooter coupling unconfirmed");
    for (Role role : List.of(Role.SHOOTER_PRIMARY, Role.SHOOTER_SECONDARY)) {
      for (String issue : device(role).issues()) result.add(role + ": " + issue);
    }
    if (coupling == Coupling.COUPLED_FOLLOWER && !followerCompatible)
      result.add("follower gearing/alignment compatibility unconfirmed");
    return result;
  }

  public List<String> automaticIssues() {
    var result = new ArrayList<>(deviceIssues());
    result.addAll(shooterIssues());
    for (Role role : Role.values())
      if (!device(role).directionConfirmed()) result.add(role + ": direction unconfirmed");
    if (!pivot.travelKnown()) result.add("pivot maximum travel missing");
    if (!pivot.profileReady()) result.add("pivot gains/profile limits missing or invalid");
    if (sequence == null) result.add("automatic sequence settings missing or invalid");
    else {
      if (sequence.minDegrees() != 0
          || sequence.maxDegrees() > pivot.maxDegrees()
          || sequence.feedDegrees() <= 0
          || sequence.intakeDegrees() <= sequence.feedDegrees())
        result.add(
            "automatic angles must be inside measured travel, with feed clear of upper stop and intake below feed");
      if (coupling == Coupling.COUPLED_FOLLOWER
          && sequence.shooterPrimaryMotorRpm() != sequence.shooterSecondaryMotorRpm())
        result.add("equal-ratio follower requires equal motor RPM targets");
    }
    if (device(Role.SHOOTER_PRIMARY).kP() == 0 && device(Role.SHOOTER_PRIMARY).kV() == 0)
      result.add("primary shooter gains missing");
    if (coupling == Coupling.INDEPENDENT
        && device(Role.SHOOTER_SECONDARY).kP() == 0
        && device(Role.SHOOTER_SECONDARY).kV() == 0) result.add("secondary shooter gains missing");
    return result.stream().distinct().toList();
  }

  public List<String> autoIssues() {
    var result = new ArrayList<>(automaticIssues());
    result.addAll(auto.issues());
    if (sequence != null
        && (auto.startDegrees() < sequence.minDegrees()
            || auto.startDegrees() > sequence.maxDegrees()))
      result.add("auto starting angle outside travel");
    return result;
  }

  public static FuelConfiguration unconfigured() {
    return new FuelConfiguration(
        Map.of(), Coupling.UNCONFIRMED, false, false, Pivot.missing(), null, Auto.missing());
  }

  public static FuelConfiguration simulation(Settings settings) {
    var devices = new java.util.EnumMap<Role, Device>(Role.class);
    for (Role role : Role.values())
      devices.put(role, new Device(20, false, true, 1, 6, 1, .001, .002));
    return new FuelConfiguration(
        devices,
        Coupling.INDEPENDENT,
        false,
        false,
        new Pivot(.1, .01, 0, 90, 180, settings.maxDegrees()),
        settings,
        new Auto(true, settings.feedDegrees(), -2, -2, .5, .2, 2, 10));
  }

  public static boolean finitePositive(double value) {
    return Double.isFinite(value) && value > 0;
  }
}
