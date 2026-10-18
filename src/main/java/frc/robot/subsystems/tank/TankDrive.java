package frc.robot.subsystems.tank;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.drive.TankDriveConstants;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** Command-owned differential drivetrain, independent of hardware and replay mode. */
public final class TankDrive extends SubsystemBase {
  private final TankDriveIO io;
  private final TankDriveIOInputsAutoLogged inputs = new TankDriveIOInputsAutoLogged();
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(TankDriveConstants.SIM_TRACK_WIDTH_METERS);
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
  private final LoggedTunableNumber autoKS = new LoggedTunableNumber("Drive/Auto/kS", 0.2);
  private final LoggedTunableNumber autoKV = new LoggedTunableNumber("Drive/Auto/kV", 2.15);
  private final LoggedTunableNumber autoKA = new LoggedTunableNumber("Drive/Auto/kA", 0.25);
  private final LoggedTunableNumber autoKP = new LoggedTunableNumber("Drive/Auto/kP", 2.0);

  public TankDrive(TankDriveIO io) {
    this.io = io;
    configureAutoBuilder();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("TankDrive", inputs);
    if (inputs.hasPositionFeedback) {
      odometry.update(
          Rotation2d.fromRadians(inputs.headingRadians),
          inputs.leftPositionMeters,
          inputs.rightPositionMeters);
    }
    if (DriverStation.isDisabled()) stop();
    Logger.recordOutput("TankDrive/Pose", getPose());
  }

  private void configureAutoBuilder() {
    if (AutoBuilder.isConfigured()) return;
    try {
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotRelativeSpeeds,
          this::runPathVelocity,
          new PPLTVController(0.02, 6.0),
          RobotConfig.fromGUISettings(),
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this);
    } catch (Exception exception) {
      DriverStation.reportError(
          "Failed to configure PathPlanner: " + exception.getMessage(), exception.getStackTrace());
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    stop();
    if (Constants.currentMode == Constants.Mode.SIM) {
      io.resetSimulationPose(pose);
      odometry.resetPosition(pose.getRotation(), 0.0, 0.0, pose);
    } else {
      odometry.resetPosition(
          Rotation2d.fromRadians(inputs.headingRadians),
          inputs.leftPositionMeters,
          inputs.rightPositionMeters,
          pose);
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
        new edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds(
            inputs.leftVelocityMetersPerSecond, inputs.rightVelocityMetersPerSecond));
  }

  private void runPathVelocity(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    if (!inputs.hasPositionFeedback) {
      stop();
      return;
    }
    var targets = kinematics.toWheelSpeeds(speeds);
    double[] accelerations = feedforwards.accelerationsMPSSq();
    double leftAccel = accelerations.length > 0 ? accelerations[0] : 0.0;
    double rightAccel = accelerations.length > 1 ? accelerations[1] : 0.0;
    double leftVolts =
        velocityVolts(targets.leftMetersPerSecond, inputs.leftVelocityMetersPerSecond, leftAccel);
    double rightVolts =
        velocityVolts(
            targets.rightMetersPerSecond, inputs.rightVelocityMetersPerSecond, rightAccel);
    setVoltage(leftVolts, rightVolts);
    Logger.recordOutput(
        "TankDrive/AutoTargetMetersPerSecond",
        new double[] {targets.leftMetersPerSecond, targets.rightMetersPerSecond});
  }

  private double velocityVolts(double target, double measured, double acceleration) {
    if (!Double.isFinite(target) || !Double.isFinite(measured) || !Double.isFinite(acceleration)) {
      return 0.0;
    }
    double staticVolts = Math.abs(target) > 1e-4 ? Math.copySign(autoKS.get(), target) : 0.0;
    return staticVolts
        + autoKV.get() * target
        + autoKA.get() * acceleration
        + autoKP.get() * (target - measured);
  }

  public void tankDrive(double left, double right) {
    var speeds = DifferentialDrive.tankDriveIK(deadband(left), deadband(right), true);
    setVoltage(
        speeds.left * TankDriveConstants.MAX_VOLTS, speeds.right * TankDriveConstants.MAX_VOLTS);
  }
  /** Driver arcade inputs: forward and counterclockwise turn, with deadband and squaring. */
  public void arcadeDriveManual(double forward, double turn) {
    var speeds = DifferentialDrive.arcadeDriveIK(deadband(forward), deadband(turn), true);
    setVoltage(
        speeds.left * TankDriveConstants.MAX_VOLTS, speeds.right * TankDriveConstants.MAX_VOLTS);
  }

  /**
   * Normalized forward and counterclockwise turn, matching the original vision adapter convention.
   */
  public void arcadeDrive(double forward, double turn) {
    var speeds = DifferentialDrive.arcadeDriveIK(sanitize(forward), sanitize(turn), false);
    setVoltage(
        speeds.left * TankDriveConstants.MAX_VOLTS, speeds.right * TankDriveConstants.MAX_VOLTS);
  }

  public void setVoltage(double left, double right) {
    if (DriverStation.isDisabled() || !Double.isFinite(left) || !Double.isFinite(right)) {
      stop();
      return;
    }
    double leftVolts =
        MathUtil.clamp(left, -TankDriveConstants.MAX_VOLTS, TankDriveConstants.MAX_VOLTS);
    double rightVolts =
        MathUtil.clamp(right, -TankDriveConstants.MAX_VOLTS, TankDriveConstants.MAX_VOLTS);
    io.setVoltage(leftVolts, rightVolts);
    Logger.recordOutput("TankDrive/CommandedVolts", new double[] {leftVolts, rightVolts});
  }

  public void stop() {
    io.stop();
    Logger.recordOutput("TankDrive/CommandedVolts", new double[] {0.0, 0.0});
  }

  private static double sanitize(double value) {
    return Double.isFinite(value) ? MathUtil.clamp(value, -1.0, 1.0) : 0.0;
  }

  private static double deadband(double value) {
    return MathUtil.applyDeadband(sanitize(value), TankDriveConstants.DEADBAND);
  }
}
