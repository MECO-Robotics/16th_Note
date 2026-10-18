package frc.robot.subsystems.motor;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;

/** Sensor units are mechanism rotations and mechanism RPM. */
public final class MotorIOSparkMax implements MotorIO {
  private final SparkMax motor;
  private boolean configured;

  public MotorIOSparkMax(int canId, double reduction, boolean brake, boolean inverted) {
    motor = new SparkMax(canId, MotorType.kBrushless);
    var config = new SparkMaxConfig();
    config
        .inverted(inverted)
        .disableFollowerMode()
        .smartCurrentLimit(60)
        .idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast)
        .voltageCompensation(12);
    config
        .encoder
        .positionConversionFactor(1.0 / reduction)
        .velocityConversionFactor(1.0 / reduction);
    boolean ready = false;
    for (int attempt = 0; attempt < 5 && !ready; attempt++) {
      ready =
          motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters)
              == REVLibError.kOk;
    }
    configured = ready;
    motor.stopMotor();
    if (!ready) DriverStation.reportError("Motor configuration failed: CAN " + canId, false);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    inputs.positionRotations = motor.getEncoder().getPosition();
    boolean positionOk = motor.getLastError() == REVLibError.kOk;
    inputs.velocityRpm = motor.getEncoder().getVelocity();
    boolean velocityOk = motor.getLastError() == REVLibError.kOk;
    double bus = motor.getBusVoltage();
    boolean busOk = motor.getLastError() == REVLibError.kOk;
    inputs.appliedVolts = bus * motor.getAppliedOutput();
    inputs.currentAmps = motor.getOutputCurrent();
    var warnings = motor.getStickyWarnings();
    boolean warningsOk = motor.getLastError() == REVLibError.kOk;
    inputs.resetDetected = warnings.hasReset || warnings.brownout;
    if (inputs.resetDetected) motor.clearFaults();
    inputs.connected = configured && positionOk && velocityOk && busOk && warningsOk && bus > 1;
  }

  @Override
  public void setVoltage(double volts) {
    if (configured) motor.setVoltage(volts);
    else motor.stopMotor();
  }
}
