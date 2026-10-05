package frc.robot.sim.fuel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.fuel.FuelConstants.Settings;
import frc.robot.subsystems.fuel.FuelIO;
import frc.robot.subsystems.fuel.FuelSequencer.Output;

/**
 * Kinematic software test model; it does not predict torque, gravity, ball motion or shot range.
 */
public final class FuelIOSim implements FuelIO {
  private static final double LOOP_SECONDS = 0.02;
  private static final double PIVOT_DEGREES_PER_SECOND = 90;
  private static final double SHOOTER_RPM_PER_SECOND = 4000;
  private Settings settings;
  private boolean referenced = true;
  private double pivotJogVolts;
  private final boolean dashboardFaults;
  private double pivotDegrees;
  private double shooterPrimaryMotorRpm;
  private double shooterSecondaryMotorRpm;
  private Output output = Output.stopped();

  public FuelIOSim(Settings settings, boolean dashboardFaults) {
    this.settings = settings;
    this.dashboardFaults = dashboardFaults;
    pivotDegrees = settings.feedDegrees();
    if (dashboardFaults) {
      for (String key :
          new String[] {
            "Disconnect",
            "LoseReference",
            "JamPivot",
            "JamShooter",
            "JamShooterSecondary",
            "DisconnectShooterPrimary",
            "DisconnectShooterSecondary",
            "DisconnectIndexer",
            "StaleFeedback"
          }) {
        SmartDashboard.putBoolean("Fuel/Sim/" + key, false);
      }
    }
  }

  @Override
  public void updateInputs(FuelIOInputs inputs) {
    if (pivotJogVolts != 0 && !fault("JamPivot")) {
      pivotDegrees =
          MathUtil.clamp(
              pivotDegrees + pivotJogVolts * 15 * LOOP_SECONDS, 0, settings.maxDegrees());
    }
    if (output.pivotEnabled() && !fault("JamPivot")) {
      pivotDegrees +=
          MathUtil.clamp(
              output.pivotDegrees() - pivotDegrees,
              -PIVOT_DEGREES_PER_SECOND * LOOP_SECONDS,
              PIVOT_DEGREES_PER_SECOND * LOOP_SECONDS);
    }
    double rpmGoal = fault("JamShooter") ? 0 : output.shooterPrimaryMotorRpm();
    shooterPrimaryMotorRpm +=
        MathUtil.clamp(
            rpmGoal - shooterPrimaryMotorRpm,
            -SHOOTER_RPM_PER_SECOND * LOOP_SECONDS,
            SHOOTER_RPM_PER_SECOND * LOOP_SECONDS);
    double secondaryGoal =
        fault("JamShooter") || fault("JamShooterSecondary") ? 0 : output.shooterSecondaryMotorRpm();
    shooterSecondaryMotorRpm +=
        MathUtil.clamp(
            secondaryGoal - shooterSecondaryMotorRpm,
            -SHOOTER_RPM_PER_SECOND * LOOP_SECONDS,
            SHOOTER_RPM_PER_SECOND * LOOP_SECONDS);
    inputs.shooterPrimaryConnected = !fault("DisconnectShooterPrimary");
    inputs.shooterSecondaryConnected = !fault("DisconnectShooterSecondary");
    inputs.indexerConnected = !fault("DisconnectIndexer");
    inputs.available = true;
    inputs.healthy = !fault("Disconnect");
    inputs.referenced = referenced && !fault("LoseReference");
    inputs.referenceStatus = inputs.referenced ? "Simulated reference valid" : "Not referenced";
    inputs.configurationStatus = "Simulation settings";
    inputs.pivotDegrees = pivotDegrees;
    inputs.shooterPrimaryMotorRpm = shooterPrimaryMotorRpm;
    inputs.shooterSecondaryMotorRpm = shooterSecondaryMotorRpm;
    inputs.ageSeconds = fault("StaleFeedback") ? settings.maxFeedbackAgeSeconds() * 2 : 0;
    inputs.intakeAppliedDuty = output.intakeDuty();
    inputs.indexerAppliedDuty = output.indexerDuty();
  }

  @Override
  public void apply(Output requested) {
    pivotJogVolts = 0;
    if (!Double.isFinite(requested.pivotDegrees())
        || !Double.isFinite(requested.shooterPrimaryMotorRpm())
        || !Double.isFinite(requested.shooterSecondaryMotorRpm())
        || !Double.isFinite(requested.indexerDuty())
        || !Double.isFinite(requested.intakeDuty())) {
      output = Output.stopped();
      return;
    }
    output =
        new Output(
            requested.pivotEnabled(),
            MathUtil.clamp(requested.pivotDegrees(), settings.minDegrees(), settings.maxDegrees()),
            MathUtil.clamp(
                requested.shooterPrimaryMotorRpm(), 0, settings.shooterPrimaryMotorRpm()),
            MathUtil.clamp(
                requested.shooterSecondaryMotorRpm(), 0, settings.shooterSecondaryMotorRpm()),
            MathUtil.clamp(requested.intakeDuty(), -1, 1),
            MathUtil.clamp(requested.indexerDuty(), -1, 1));
  }

  @Override
  public boolean configure(frc.robot.constants.fuel.FuelConfiguration config) {
    if (!edu.wpi.first.wpilibj.DriverStation.isDisabled()) return false;
    stop();
    referenced = false;
    if (config.sequence() == null) return false;
    settings = config.sequence();
    return true;
  }

  @Override
  public boolean confirmUpperReference() {
    if (!edu.wpi.first.wpilibj.DriverStation.isDisabled() || fault("Disconnect")) return false;
    stop();
    pivotDegrees = 0;
    referenced = true;
    return true;
  }

  @Override
  public void invalidateReference(String reason) {
    referenced = false;
  }

  @Override
  public boolean commissioningReady(
      frc.robot.subsystems.fuel.FuelCommissioning.Selection selection) {
    return !fault("Disconnect")
        && (selection != frc.robot.subsystems.fuel.FuelCommissioning.Selection.PIVOT || referenced);
  }

  @Override
  public void commissioning(
      frc.robot.subsystems.fuel.FuelCommissioning.Selection selection, double volts) {
    stop();
    if (!edu.wpi.first.wpilibj.DriverStation.isTestEnabled() || !Double.isFinite(volts)) return;
    volts = MathUtil.clamp(volts, -1, 1);
    switch (selection) {
      case PIVOT -> pivotJogVolts = volts;
      case ROLLERS -> output = new Output(false, 0, 0, 0, volts / 12, 0);
      case INDEXER -> output = new Output(false, 0, 0, 0, 0, volts / 12);
      case SHOOTERS -> output =
          new Output(false, 0, Math.abs(volts) * 500, Math.abs(volts) * 500, 0, 0);
      default -> {}
    }
  }

  @Override
  public void stop() {
    pivotJogVolts = 0;
    output = Output.stopped();
  }

  private boolean fault(String key) {
    return dashboardFaults && SmartDashboard.getBoolean("Fuel/Sim/" + key, false);
  }
}
