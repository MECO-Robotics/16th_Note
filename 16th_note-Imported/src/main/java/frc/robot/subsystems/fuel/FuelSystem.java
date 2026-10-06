package frc.robot.subsystems.fuel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.fuel.FuelConfiguration;
import frc.robot.constants.fuel.FuelConstants;
import frc.robot.constants.fuel.FuelConstants.Settings;
import frc.robot.controls.DriverControls;
import frc.robot.sim.fuel.FuelIOSim;
import frc.robot.subsystems.tank.TankDrive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Single owner for teleop, autonomous and commissioning. All paths converge on the same IO. */
public final class FuelSystem extends SubsystemBase {
  private enum Request {
    NONE,
    TELEOP,
    AUTONOMOUS,
    COMMISSIONING
  }

  private static final double REQUEST_TIMEOUT_SECONDS = .1;
  private final FuelIO io;
  private final FuelIOInputsAutoLogged inputs = new FuelIOInputsAutoLogged();
  private FuelSequencer sequence;
  private FuelConfiguration configuration;
  private final DoubleSupplier clock;
  private FuelDashboard dashboard;
  private final FuelCommissioning commissioning = new FuelCommissioning();
  private Request request = Request.NONE;
  private boolean controllerConnected, intakeRequested, shootRequested;
  private FuelCommissioning.Selection selection = FuelCommissioning.Selection.NONE;
  private double testVolts, lastRequest;
  private FuelSequencer.Output output = FuelSequencer.Output.stopped();
  private String actionStatus = "";

  public static FuelSystem create() {
    FuelConfiguration config =
        Constants.currentMode == Constants.Mode.REAL
            ? FuelConstants.REAL
            : FuelConfiguration.simulation(FuelConstants.SIMULATION);
    FuelIO io =
        switch (Constants.currentMode) {
          case REAL -> new FuelIOSparkMax(config);
          case SIM -> new FuelIOSim(FuelConstants.SIMULATION, true);
          case REPLAY -> new FuelIO() {};
        };
    if (Constants.currentMode == Constants.Mode.SIM)
      io.invalidateReference("Startup: confirm simulated upper reference");
    var system = new FuelSystem(io, config, Timer::getFPGATimestamp);
    system.dashboard = new FuelDashboard(config);
    SmartDashboard.putString("Fuel/Backend", Constants.currentMode.name());
    SmartDashboard.putData(
        "Fuel/Apply configuration (disabled)",
        Commands.runOnce(() -> system.applyConfiguration(system.dashboard.read()), system)
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Fuel/Confirm intake at upper stop",
        Commands.runOnce(system::confirmUpperReference, system).ignoringDisable(true));
    return system;
  }

  public FuelSystem(FuelIO io, FuelConfiguration configuration, DoubleSupplier clock) {
    this.io = io;
    this.configuration = configuration;
    this.clock = clock;
    sequence =
        configuration.sequence() == null ? null : new FuelSequencer(configuration.sequence());
  }
  /** Injection constructor for sequence-only tests; production always supplies a configuration. */
  public FuelSystem(FuelIO io, Settings settings, DoubleSupplier clock) {
    this.io = io;
    this.clock = clock;
    sequence = new FuelSequencer(settings);
  }

  public Command teleopCommand(DriverControls controls) {
    return runEnd(
            () ->
                request(
                    controls.isConnected(),
                    controls.isIntakeRequested(),
                    controls.isShootRequested()),
            this::stop)
        .withName("Fuel teleop sequence");
  }

  public Command commissioningCommand(DriverControls controls, TankDrive drive) {
    return Commands.runEnd(
            () -> {
              drive.stop();
              requestCommissioning(
                  controls.isConnected(),
                  controls.isShootRequested(),
                  dashboard.selection(),
                  dashboard.testVolts());
            },
            () -> {
              stop();
              drive.stop();
            },
            this,
            drive)
        .onlyWhile(DriverStation::isTestEnabled)
        .withName("Fuel commissioning");
  }

  public void request(boolean connected, boolean intake, boolean shoot) {
    request = Request.TELEOP;
    controllerConnected = connected;
    intakeRequested = intake;
    shootRequested = shoot;
    lastRequest = clock.getAsDouble();
  }

  public void requestAutonomous(boolean shoot) {
    request = Request.AUTONOMOUS;
    controllerConnected = true;
    intakeRequested = false;
    shootRequested = shoot;
    lastRequest = clock.getAsDouble();
  }

  public void requestCommissioning(
      boolean connected, boolean held, FuelCommissioning.Selection selection, double volts) {
    request = Request.COMMISSIONING;
    controllerConnected = connected;
    shootRequested = held;
    intakeRequested = false;
    this.selection = selection;
    testVolts = volts;
    lastRequest = clock.getAsDouble();
  }

  /** Practice presets cannot write real hardware settings or change an enabled robot. */
  public boolean setSimulationShooterTarget(double motorRpm) {
    if (!(io instanceof FuelIOSim)
        || !DriverStation.isDisabled()
        || configuration == null
        || configuration.sequence() == null
        || !Double.isFinite(motorRpm)
        || motorRpm <= 0
        || motorRpm > 6000) return false;
    var c = configuration;
    var settings = c.sequence().withShooterMotorRpm(motorRpm, motorRpm);
    if (!applyConfiguration(
        new FuelConfiguration(
            c.devices(),
            c.coupling(),
            c.followerCompatible(),
            c.followerInverted(),
            c.pivot(),
            settings,
            c.auto()))) return false;
    SmartDashboard.putNumber("Fuel/Config/PrimaryMotorRpm", motorRpm);
    SmartDashboard.putNumber("Fuel/Config/SecondaryMotorRpm", motorRpm);
    return true;
  }

  public boolean applyConfiguration(FuelConfiguration config) {
    if (!DriverStation.isDisabled()) {
      actionStatus = "Configuration rejected: disable first";
      return false;
    }
    stop();
    configuration = config;
    sequence = config.sequence() == null ? null : new FuelSequencer(config.sequence());
    boolean applied = io.configure(config);
    Logger.recordOutput("Fuel/ActiveConfiguration", config.toString());
    SmartDashboard.putString("Fuel/ActiveConfiguration", config.toString());
    actionStatus =
        applied
            ? "Configuration applied; reference intake again"
            : "Some devices are unconfigured or failed; inspect diagnostics";
    return applied;
  }

  public boolean confirmUpperReference() {
    if (!DriverStation.isDisabled()) {
      actionStatus = "Reference rejected: disable first";
      return false;
    }
    stop();
    boolean accepted = io.confirmUpperReference();
    actionStatus =
        accepted
            ? "Reference requested; awaiting verification"
            : "Reference rejected: inspect pivot configuration and health";
    return accepted;
  }

  public String automaticBlockReason() {
    if (configuration != null && !configuration.automaticIssues().isEmpty())
      return String.join("; ", configuration.automaticIssues());
    if (!inputs.available) return "Required devices unavailable/configuration incomplete";
    if (!inputs.healthy
        || !inputs.shooterPrimaryConnected
        || !inputs.shooterSecondaryConnected
        || !inputs.indexerConnected) return "Required controller/sensor unhealthy";
    if (!inputs.referenced) return "Intake not referenced";
    if (!fault().isEmpty()) return fault();
    return "";
  }

  public String autonomousBlockReason() {
    String reason = automaticBlockReason();
    if (!reason.isEmpty()) return reason;
    if (configuration == null) return "Autonomous configuration missing";
    if (!configuration.autoIssues().isEmpty()) return String.join("; ", configuration.autoIssues());
    if (!Double.isFinite(inputs.pivotDegrees)
        || Math.abs(inputs.pivotDegrees - configuration.auto().startDegrees())
            > configuration.sequence().positionToleranceDegrees())
      return "Intake not at verified autonomous starting position";
    return "";
  }

  public FuelConfiguration.Auto autonomousConfiguration() {
    return configuration == null ? FuelConfiguration.Auto.missing() : configuration.auto();
  }

  public void armAutonomous() {
    if (sequence != null) sequence.armAutonomous(inputs.pivotDegrees, clock.getAsDouble());
  }

  public double pivotDegrees() {
    return inputs.pivotDegrees;
  }

  public double simulatedShooterRpm() {
    return Math.min(inputs.shooterPrimaryMotorRpm, inputs.shooterSecondaryMotorRpm);
  }

  public boolean collecting() {
    return output.intakeDuty() > 0
        && configuration != null
        && configuration.sequence() != null
        && Math.abs(inputs.pivotDegrees - configuration.sequence().intakeDegrees())
            <= configuration.sequence().positionToleranceDegrees();
  }

  public boolean feeding() {
    return output.indexerDuty() > 0;
  }

  @Override
  public void periodic() {
    double now = clock.getAsDouble();
    io.updateInputs(inputs);
    FuelMode mode = FuelMode.current();
    inputs.timestampSeconds = now;
    inputs.enabled = mode == FuelMode.TELEOP || mode == FuelMode.AUTONOMOUS;
    inputs.commandActive =
        request != Request.NONE
            && now - lastRequest >= 0
            && now - lastRequest <= REQUEST_TIMEOUT_SECONDS;
    inputs.controllerConnected = controllerConnected;
    inputs.brownedOut = RobotController.isBrownedOut();
    inputs.intakeRequested = intakeRequested;
    inputs.shootRequested = shootRequested;
    inputs.requestMode = request.name();
    Logger.processInputs("Fuel/Inputs", inputs);
    Request loggedRequest;
    try {
      loggedRequest = Request.valueOf(inputs.requestMode);
    } catch (IllegalArgumentException e) {
      loggedRequest = Request.NONE;
    }
    if (inputs.brownedOut) io.invalidateReference("Robot brownout");
    if (mode == FuelMode.TEST && loggedRequest == Request.COMMISSIONING) {
      if (sequence != null) sequence.stop();
      output = FuelSequencer.Output.stopped();
      double volts =
          commissioning.update(
              now,
              inputs.commandActive && !inputs.brownedOut,
              inputs.controllerConnected,
              inputs.shootRequested,
              selection,
              testVolts,
              io.commissioningReady(selection));
      if (volts == 0) io.stop();
      else io.commissioning(selection, volts);
    } else {
      commissioning.reset();
      boolean matches =
          (mode == FuelMode.TELEOP && loggedRequest == Request.TELEOP)
              || (mode == FuelMode.AUTONOMOUS && loggedRequest == Request.AUTONOMOUS);
      if (sequence == null) output = FuelSequencer.Output.stopped();
      else if (mode == FuelMode.DISABLED) {
        output = sequence.update(now, false, false, false, false, false, feedback());
      } else if (!inputs.commandActive || !matches) {
        sequence.stop();
        output = FuelSequencer.Output.stopped();
      } else if (configuration != null && !configuration.automaticIssues().isEmpty()) {
        sequence.stop();
        output = FuelSequencer.Output.stopped();
      } else
        output =
            sequence.update(
                inputs.timestampSeconds,
                true,
                loggedRequest == Request.AUTONOMOUS || inputs.controllerConnected,
                inputs.brownedOut,
                inputs.intakeRequested,
                inputs.shootRequested,
                feedback());
      io.apply(output);
    }
    publish();
  }

  private FuelSequencer.Feedback feedback() {
    return new FuelSequencer.Feedback(
        inputs.available,
        inputs.healthy
            && inputs.shooterPrimaryConnected
            && inputs.shooterSecondaryConnected
            && inputs.indexerConnected,
        inputs.referenced,
        inputs.pivotDegrees,
        inputs.shooterPrimaryMotorRpm,
        inputs.shooterSecondaryMotorRpm,
        inputs.ageSeconds);
  }

  private void publish() {
    Logger.recordOutput("Fuel/State", state().name());
    Logger.recordOutput("Fuel/Fault", fault());
    Logger.recordOutput("Fuel/PivotEnabled", output.pivotEnabled());
    Logger.recordOutput("Fuel/PivotGoalDegrees", output.pivotDegrees());
    Logger.recordOutput("Fuel/ShooterPrimaryGoalMotorRpm", output.shooterPrimaryMotorRpm());
    Logger.recordOutput("Fuel/ShooterSecondaryGoalMotorRpm", output.shooterSecondaryMotorRpm());
    Logger.recordOutput("Fuel/IntakeDuty", output.intakeDuty());
    Logger.recordOutput("Fuel/IndexerDuty", output.indexerDuty());
    SmartDashboard.putString("Fuel/State", state().name());
    SmartDashboard.putString("Fuel/Fault", fault());
    SmartDashboard.putString("Fuel/AutomaticBlockedBy", automaticBlockReason());
    SmartDashboard.putString("Fuel/AutoBlockedBy", autonomousBlockReason());
    SmartDashboard.putString("Fuel/Devices", inputs.configurationStatus);
    SmartDashboard.putString("Fuel/Reference", inputs.referenceStatus);
    SmartDashboard.putString("Fuel/ActionStatus", actionStatus);
    SmartDashboard.putString("Fuel/Test/Status", commissioning.status());
    if (dashboard != null)
      SmartDashboard.putString("Fuel/ConfigurationParseStatus", dashboard.parseStatus());
    SmartDashboard.putBoolean("Fuel/DevicesConfigured", inputs.available);
    SmartDashboard.putBoolean("Fuel/AutomaticReady", automaticBlockReason().isEmpty());
    SmartDashboard.putBoolean("Fuel/AutonomousReady", autonomousBlockReason().isEmpty());
    SmartDashboard.putBoolean("Fuel/Available", inputs.available);
    SmartDashboard.putBoolean("Fuel/Referenced", inputs.referenced);
    SmartDashboard.putNumber("Fuel/PivotDegrees", inputs.pivotDegrees);
    SmartDashboard.putNumber("Fuel/ShooterPrimaryMotorRpm", inputs.shooterPrimaryMotorRpm);
    SmartDashboard.putNumber("Fuel/ShooterSecondaryMotorRpm", inputs.shooterSecondaryMotorRpm);
    SmartDashboard.putNumber("Fuel/IndexerDuty", output.indexerDuty());
  }

  public void stop() {
    request = Request.NONE;
    intakeRequested = false;
    shootRequested = false;
    if (sequence != null) sequence.stop();
    commissioning.reset();
    output = FuelSequencer.Output.stopped();
    io.stop();
  }

  public FuelSequencer.State state() {
    return sequence == null ? FuelSequencer.State.UNAVAILABLE : sequence.state();
  }

  public String fault() {
    return sequence == null ? "" : sequence.fault();
  }
}
