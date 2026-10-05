package frc.robot.subsystems.fuel;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.fuel.FuelConfiguration.Device;
import frc.robot.constants.fuel.FuelConfiguration.Role;

/** REVLib 2026 adapter. Cached getter success is not a timestamp for a new CAN frame. */
final class SparkFuelMotor implements FuelMotor {
  private final SparkMax motor;
  private final Role role;
  private boolean ok;

  SparkFuelMotor(Role role) {
    this.role = role;
    motor = new SparkMax(role.canId, MotorType.kBrushless);
    motor.stopMotor();
    motor.setPeriodicFrameTimeout(100);
    motor.setCANTimeout(20);
  }

  @Override
  public boolean configure(Device device, Integer leader, boolean followInverted) {
    var config = new SparkMaxConfig();
    config
        .smartCurrentLimit(device.currentAmps())
        .inverted(device.inverted())
        .idleMode(role == Role.PIVOT ? IdleMode.kBrake : IdleMode.kCoast)
        .voltageCompensation(12);
    config.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    config
        .closedLoop
        .pid(device.kP(), 0, 0)
        .outputRange(-device.maxVolts() / 12, device.maxVolts() / 12);
    config.closedLoop.feedForward.kV(device.kV());
    config.softLimit.forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .faultsAlwaysOn(true)
        .faultsPeriodMs(20)
        .warningsAlwaysOn(true)
        .warningsPeriodMs(20)
        .appliedOutputPeriodMs(100)
        .busVoltagePeriodMs(100)
        .outputCurrentPeriodMs(100)
        .motorTemperaturePeriodMs(500);
    if (leader != null) config.follow(leader, followInverted);
    else config.disableFollowerMode();
    return motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters)
        == REVLibError.kOk;
  }

  private double checked(java.util.function.DoubleSupplier getter) {
    double value = getter.getAsDouble();
    ok &= motor.getLastError() == REVLibError.kOk && Double.isFinite(value);
    return value;
  }

  @Override
  public Sample read() {
    ok = true;
    double position = checked(() -> motor.getEncoder().getPosition());
    double rpm = checked(() -> motor.getEncoder().getVelocity());
    double current = checked(motor::getOutputCurrent);
    double duty = checked(motor::getAppliedOutput);
    double volts = duty * checked(motor::getBusVoltage);
    double temperature = checked(motor::getMotorTemperature);
    var warnings = motor.getStickyWarnings();
    ok &= motor.getLastError() == REVLibError.kOk;
    var faults = motor.getFaults();
    ok &= motor.getLastError() == REVLibError.kOk;
    return new Sample(
        ok,
        position,
        rpm,
        current,
        volts,
        temperature,
        warnings.hasReset,
        warnings.brownout,
        faults.rawBits,
        duty);
  }

  @Override
  public boolean clearFaults() {
    return motor.clearFaults() == REVLibError.kOk;
  }

  @Override
  public boolean zeroEncoder() {
    return motor.getEncoder().setPosition(0) == REVLibError.kOk;
  }

  @Override
  public boolean softLimits(double min, double max, boolean minEnabled, boolean maxEnabled) {
    var config = new SparkMaxConfig();
    config
        .softLimit
        .reverseSoftLimit(min)
        .reverseSoftLimitEnabled(minEnabled)
        .forwardSoftLimit(max)
        .forwardSoftLimitEnabled(maxEnabled);
    return motor.configure(
            config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
        == REVLibError.kOk;
  }

  @Override
  public void voltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void velocity(double rpm) {
    motor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
