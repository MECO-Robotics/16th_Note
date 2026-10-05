package frc.robot.subsystems.fuel;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.fuel.FuelConfiguration;
import frc.robot.constants.fuel.FuelConfiguration.*;
import frc.robot.constants.fuel.FuelConstants.Settings;
import java.util.EnumMap;

/** Staged tuning. Only an explicit disabled apply replaces the immutable active snapshot. */
public final class FuelDashboard {
  private static final String ROOT = "Fuel/Config/";
  private final SendableChooser<Coupling> coupling = new SendableChooser<>();
  private final SendableChooser<FuelCommissioning.Selection> selection = new SendableChooser<>();
  private String parseStatus = "";

  public FuelDashboard(FuelConfiguration initial) {
    coupling.setDefaultOption(initial.coupling().name(), initial.coupling());
    for (Coupling value : Coupling.values())
      if (value != initial.coupling()) coupling.addOption(value.name(), value);
    SmartDashboard.putData(ROOT + "ShooterCoupling", coupling);
    selection.setDefaultOption("None", FuelCommissioning.Selection.NONE);
    for (var value : FuelCommissioning.Selection.values())
      if (value != FuelCommissioning.Selection.NONE) selection.addOption(value.name(), value);
    SmartDashboard.putData("Fuel/Test/Mechanism", selection);
    SmartDashboard.putNumber("Fuel/Test/Volts", 0);
    for (Role role : Role.values()) {
      Device d = initial.device(role);
      String key = role.name() + "/";
      put(key + "CurrentAmps", d.currentAmps());
      put(key + "MaxVolts", d.maxVolts());
      put(key + "TestVoltsCap", d.testVolts());
      put(key + "FeedbackSign", d.feedbackSign());
      put(key + "kP", d.kP());
      put(key + "kV", d.kV());
      flag(key + "Inverted", d.inverted());
      flag(key + "DirectionConfirmed", d.directionConfirmed());
    }
    flag("FollowerCompatible", initial.followerCompatible());
    flag("FollowerInverted", initial.followerInverted());
    var p = initial.pivot();
    put("Pivot/kP", p.kP());
    put("Pivot/kD", p.kD());
    put("Pivot/GravityVolts", p.gravityVolts());
    put("Pivot/MaxVelocity", p.maxDegreesPerSecond());
    put("Pivot/MaxAcceleration", p.maxDegreesPerSecondSquared());
    put("Pivot/MaxDegrees", p.maxDegrees());
    Settings s = initial.sequence();
    put("IntakeDegrees", s == null ? 0 : s.intakeDegrees());
    put("FeedDegrees", s == null ? 0 : s.feedDegrees());
    put("AgitationDegrees", s == null ? 0 : s.agitationDegrees());
    put("PositionToleranceDegrees", s == null ? 0 : s.positionToleranceDegrees());
    put("PrimaryMotorRpm", s == null ? 0 : s.shooterPrimaryMotorRpm());
    put("SecondaryMotorRpm", s == null ? 0 : s.shooterSecondaryMotorRpm());
    put("PrimaryToleranceMotorRpm", s == null ? 0 : s.primarySpeedToleranceMotorRpm());
    put("SecondaryToleranceMotorRpm", s == null ? 0 : s.secondarySpeedToleranceMotorRpm());
    put("ReadySeconds", s == null ? 0 : s.readySeconds());
    put("SpinupTimeoutSeconds", s == null ? 0 : s.spinupTimeoutSeconds());
    put("MotionTimeoutSeconds", s == null ? 0 : s.motionTimeoutSeconds());
    put("FeedDwellSeconds", s == null ? 0 : s.feedDwellSeconds());
    put("AgitationDwellSeconds", s == null ? 0 : s.agitationDwellSeconds());
    put("AgitationCycles", s == null ? 0 : s.maxAgitationCycles());
    flag("AgitationEnabled", s != null && s.agitationEnabled());
    put("IntakeDuty", s == null ? 0 : s.intakeDutyCycle());
    put("IndexerDuty", s == null ? 0 : s.indexerDutyCycle());
    Auto a = initial.auto();
    flag("Auto/Verified", a.verified());
    put("Auto/StartDegrees", a.startDegrees());
    put("Auto/LeftVolts", a.leftVolts());
    put("Auto/RightVolts", a.rightVolts());
    put("Auto/BackSeconds", a.backSeconds());
    put("Auto/SettleSeconds", a.settleSeconds());
    put("Auto/FeedSeconds", a.feedSeconds());
    put("Auto/TotalSeconds", a.totalSeconds());
  }

  private static void put(String key, double value) {
    SmartDashboard.putNumber(ROOT + key, value);
  }

  private static void flag(String key, boolean value) {
    SmartDashboard.putBoolean(ROOT + key, value);
  }

  private static double n(String key) {
    return SmartDashboard.getNumber(ROOT + key, Double.NaN);
  }

  private static boolean b(String key) {
    return SmartDashboard.getBoolean(ROOT + key, false);
  }

  private static int integer(String key) {
    double value = n(key);
    return Double.isFinite(value) && value == Math.rint(value) ? (int) value : 0;
  }

  public FuelCommissioning.Selection selection() {
    return selection.getSelected();
  }

  public double testVolts() {
    return SmartDashboard.getNumber("Fuel/Test/Volts", 0);
  }

  public String parseStatus() {
    return parseStatus;
  }

  public FuelConfiguration read() {
    var devices = new EnumMap<Role, Device>(Role.class);
    for (Role role : Role.values()) {
      String k = role.name() + "/";
      devices.put(
          role,
          new Device(
              integer(k + "CurrentAmps"),
              b(k + "Inverted"),
              b(k + "DirectionConfirmed"),
              integer(k + "FeedbackSign"),
              n(k + "MaxVolts"),
              n(k + "TestVoltsCap"),
              n(k + "kP"),
              n(k + "kV")));
    }
    Pivot pivot =
        new Pivot(
            n("Pivot/kP"),
            n("Pivot/kD"),
            n("Pivot/GravityVolts"),
            n("Pivot/MaxVelocity"),
            n("Pivot/MaxAcceleration"),
            n("Pivot/MaxDegrees"));
    Settings settings = null;
    parseStatus = "";
    try {
      settings =
          new Settings(
              0,
              pivot.maxDegrees(),
              n("IntakeDegrees"),
              n("FeedDegrees"),
              n("AgitationDegrees"),
              n("PositionToleranceDegrees"),
              n("PrimaryMotorRpm"),
              n("PrimaryToleranceMotorRpm"),
              n("ReadySeconds"),
              n("SpinupTimeoutSeconds"),
              n("MotionTimeoutSeconds"),
              n("FeedDwellSeconds"),
              n("AgitationDwellSeconds"),
              integer("AgitationCycles"),
              b("AgitationEnabled"),
              .1,
              n("IntakeDuty"),
              n("IndexerDuty"),
              n("SecondaryMotorRpm"),
              n("SecondaryToleranceMotorRpm"));
    } catch (IllegalArgumentException e) {
      var invalid = new java.util.ArrayList<String>();
      for (String key :
          new String[] {
            "Pivot/MaxDegrees",
            "IntakeDegrees",
            "FeedDegrees",
            "PositionToleranceDegrees",
            "PrimaryMotorRpm",
            "SecondaryMotorRpm",
            "PrimaryToleranceMotorRpm",
            "SecondaryToleranceMotorRpm",
            "ReadySeconds",
            "SpinupTimeoutSeconds",
            "MotionTimeoutSeconds",
            "IntakeDuty",
            "IndexerDuty"
          }) {
        if (!FuelConfiguration.finitePositive(n(key))) invalid.add(key + " must be finite and >0");
      }
      parseStatus =
          invalid.isEmpty()
              ? e.getMessage()
                  + ": verify angles, tolerances, dwell/timeout ordering and duty limits"
              : String.join("; ", invalid);
    }
    return new FuelConfiguration(
        devices,
        coupling.getSelected(),
        b("FollowerCompatible"),
        b("FollowerInverted"),
        pivot,
        settings,
        new Auto(
            b("Auto/Verified"),
            n("Auto/StartDegrees"),
            n("Auto/LeftVolts"),
            n("Auto/RightVolts"),
            n("Auto/BackSeconds"),
            n("Auto/SettleSeconds"),
            n("Auto/FeedSeconds"),
            n("Auto/TotalSeconds")));
  }
}
