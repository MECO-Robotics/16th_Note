package frc.robot.subsystems.tank;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.constants.drive.TankDriveConstants;

/** PWM Spark Max outputs with a hardware-side MotorSafety watchdog. */
public final class TankDriveIOPWM implements TankDriveIO {
  private final PWMSparkMax left = new PWMSparkMax(TankDriveConstants.LEFT_PWM_PORT);
  private final PWMSparkMax right = new PWMSparkMax(TankDriveConstants.RIGHT_PWM_PORT);

  public TankDriveIOPWM() {
    left.setInverted(TankDriveConstants.LEFT_INVERTED);
    right.setInverted(TankDriveConstants.RIGHT_INVERTED);
    left.setExpiration(0.1);
    right.setExpiration(0.1);
    left.setSafetyEnabled(true);
    right.setSafetyEnabled(true);
  }

  @Override
  public void updateInputs(TankDriveIOInputs inputs) {
    inputs.leftAppliedVolts = left.get() * RobotController.getBatteryVoltage();
    inputs.rightAppliedVolts = right.get() * RobotController.getBatteryVoltage();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
  }

  @Override
  public void stop() {
    left.stopMotor();
    right.stopMotor();
  }
}
