package frc.robot.subsystems.tank;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.drive.TankDriveConstants;
import org.littletonrobotics.junction.Logger;

/** Command-owned differential drivetrain, independent of hardware and replay mode. */
public final class TankDrive extends SubsystemBase {
  private final TankDriveIO io;
  private final TankDriveIOInputsAutoLogged inputs = new TankDriveIOInputsAutoLogged();

  public TankDrive(TankDriveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      stop();
    }
    io.updateInputs(inputs);
    Logger.processInputs("TankDrive", inputs);
  }

  public void tankDrive(double left, double right) {
    var speeds = DifferentialDrive.tankDriveIK(deadband(left), deadband(right), true);
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
    Logger.recordOutput("TankDrive/RequestedVolts", new double[] {leftVolts, rightVolts});
  }

  public void stop() {
    io.stop();
    Logger.recordOutput("TankDrive/RequestedVolts", new double[] {0.0, 0.0});
  }

  private static double sanitize(double value) {
    return Double.isFinite(value) ? MathUtil.clamp(value, -1.0, 1.0) : 0.0;
  }

  private static double deadband(double value) {
    return MathUtil.applyDeadband(sanitize(value), TankDriveConstants.DEADBAND);
  }
}
