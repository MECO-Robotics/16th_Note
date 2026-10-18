package frc.robot.subsystems.flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;

/** Single SPARK MAX variant of the rebuilt FlywheelIO with onboard MAXMotion velocity control. */
public final class FlywheelIOSparkMax implements FlywheelIO {
  private final String name;
  private final SparkMax motor;
  private final SparkMaxConfig config = new SparkMaxConfig();
  private boolean configured;
  private double target;

  public FlywheelIOSparkMax(String name, FlywheelHardwareConfig hardware) {
    this.name = name;
    motor = new SparkMax(hardware.canIds()[0], MotorType.kBrushless);
    config
        .inverted(hardware.reversed()[0])
        .disableFollowerMode()
        .smartCurrentLimit(hardware.currentLimit())
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12);
    config
        .encoder
        .positionConversionFactor(1.0 / hardware.gearRatio())
        .velocityConversionFactor(1.0 / (60.0 * hardware.gearRatio()));
    applyConfiguration();
    motor.stopMotor();
  }

  private void applyConfiguration() {
    configured = false;
    for (int attempt = 0; attempt < 5 && !configured; attempt++)
      configured =
          motor.configure(
                  config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
              == REVLibError.kOk;
    if (!configured) motor.stopMotor();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.position = motor.getEncoder().getPosition();
    boolean positionOk = motor.getLastError() == REVLibError.kOk;
    inputs.velocity = motor.getEncoder().getVelocity();
    boolean velocityOk = motor.getLastError() == REVLibError.kOk;
    double bus = motor.getBusVoltage();
    boolean busOk = motor.getLastError() == REVLibError.kOk;
    inputs.appliedVolts = bus * motor.getAppliedOutput();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.connected = configured && positionOk && velocityOk && busOk && bus > 1;
    inputs.desiredVelocity = target;
  }

  @Override
  public void setGains(FlywheelGains gains) {
    config.closedLoop.pid(gains.kP(), gains.kI(), gains.kD()).outputRange(-1, 1);
    config.closedLoop.feedForward.kS(gains.kS()).kV(gains.kV()).kA(gains.kA());
    config.closedLoop.maxMotion.maxAcceleration(gains.kMaxAccel());
    applyConfiguration();
  }

  @Override
  public void setVelocity(double velocity) {
    target = velocity;
    if (configured && Double.isFinite(velocity))
      motor.getClosedLoopController().setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
    else motor.stopMotor();
  }

  @Override
  public void setVoltage(double volts) {
    target = 0;
    if (configured && Double.isFinite(volts)) motor.setVoltage(volts);
    else motor.stopMotor();
  }

  @Override
  public String getName() {
    return name;
  }
}
