package frc.robot.subsystems.tank;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.MotorSafety;
import frc.robot.constants.drive.TankDriveConstants;

/** CAN SPARK MAX voltage control with a 100 ms command watchdog. */
public final class TankDriveIOSparkMax extends MotorSafety implements TankDriveIO {
  private final SparkMax left;
  private final SparkMax right;
  private final boolean configured;

  public TankDriveIOSparkMax(MotorType motorType) {
    left = new SparkMax(TankDriveConstants.LEFT_CAN_ID, motorType);
    right = new SparkMax(TankDriveConstants.RIGHT_CAN_ID, motorType);
    boolean leftReady = configure(left, TankDriveConstants.LEFT_INVERTED);
    boolean rightReady = configure(right, TankDriveConstants.RIGHT_INVERTED);
    configured = leftReady && rightReady;
    stop();
    setExpiration(0.1);
    setSafetyEnabled(true);
  }

  private static boolean configure(SparkMax motor, boolean inverted) {
    var config = new SparkMaxConfig();
    config.inverted(inverted).disableFollowerMode();
    for (int attempt = 0; attempt < 5; attempt++) {
      if (motor.configure(
              config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
          == REVLibError.kOk) {
        return true;
      }
    }
    DriverStation.reportError(
        "Tank SPARK MAX configuration failed: CAN " + motor.getDeviceId(), false);
    return false;
  }

  @Override
  public void updateInputs(TankDriveIOInputs inputs) {
    inputs.leftAppliedVolts = left.getAppliedOutput() * left.getBusVoltage();
    inputs.rightAppliedVolts = right.getAppliedOutput() * right.getBusVoltage();
    // Wheel distance and heading are not calibrated on this robot.
    inputs.hasPositionFeedback = false;
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    if (!configured) {
      stop();
      return;
    }
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
    feed();
  }

  @Override
  public void stop() {
    left.stopMotor();
    right.stopMotor();
    feed();
  }

  @Override
  public void stopMotor() {
    stop();
  }

  @Override
  public String getDescription() {
    return "Tank CAN SPARK MAX drivetrain";
  }
}
