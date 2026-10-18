package frc.robot.subsystems.tank;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
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
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final boolean configured;

  public TankDriveIOSparkMax(MotorType motorType) {
    left = new SparkMax(TankDriveConstants.LEFT_CAN_ID, motorType);
    right = new SparkMax(TankDriveConstants.RIGHT_CAN_ID, motorType);
    leftEncoder = left.getEncoder();
    rightEncoder = right.getEncoder();
    boolean leftReady = configure(left, TankDriveConstants.LEFT_INVERTED);
    boolean rightReady = configure(right, TankDriveConstants.RIGHT_INVERTED);
    configured = leftReady && rightReady;
    if (configured) {
      leftEncoder.setPosition(0.0);
      rightEncoder.setPosition(0.0);
    }
    stop();
    setExpiration(0.1);
    setSafetyEnabled(true);
  }

  private static boolean configure(SparkMax motor, boolean inverted) {
    var config = new SparkMaxConfig();
    config.inverted(inverted).disableFollowerMode().smartCurrentLimit(60);
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
    double metersPerMotorRotation =
        2.0
            * Math.PI
            * TankDriveConstants.SIM_WHEEL_RADIUS_METERS
            / TankDriveConstants.SIM_GEAR_RATIO;
    double leftSign = TankDriveConstants.LEFT_INVERTED ? -1.0 : 1.0;
    double rightSign = TankDriveConstants.RIGHT_INVERTED ? -1.0 : 1.0;
    inputs.hasPositionFeedback = configured;
    inputs.leftPositionMeters = leftEncoder.getPosition() * metersPerMotorRotation * leftSign;
    inputs.rightPositionMeters = rightEncoder.getPosition() * metersPerMotorRotation * rightSign;
    inputs.leftVelocityMetersPerSecond =
        leftEncoder.getVelocity() / 60.0 * metersPerMotorRotation * leftSign;
    inputs.rightVelocityMetersPerSecond =
        rightEncoder.getVelocity() / 60.0 * metersPerMotorRotation * rightSign;
    inputs.headingRadians =
        (inputs.rightPositionMeters - inputs.leftPositionMeters)
            / TankDriveConstants.SIM_TRACK_WIDTH_METERS;
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
