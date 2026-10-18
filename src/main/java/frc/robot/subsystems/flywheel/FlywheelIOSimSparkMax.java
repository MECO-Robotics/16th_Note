package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;

/** No-CAN physics adapter using the same RPS/PID/feedforward units as real FlywheelIO. */
public final class FlywheelIOSimSparkMax implements FlywheelIO {
  private final String name;
  private final DCMotorSim model;
  private final PIDController pid = new PIDController(0, 0, 0);
  private FlywheelGains gains;
  private double volts, target, profiledVelocity;
  private boolean closedLoop;

  public FlywheelIOSimSparkMax(String name, FlywheelHardwareConfig config, DCMotor motor) {
    this.name = name;
    model =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                motor, config.momentOfInertiaKgMetersSquared(), config.gearRatio()),
            motor);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop && gains != null) {
      double previous = profiledVelocity;
      profiledVelocity +=
          MathUtil.clamp(
              target - profiledVelocity, -gains.kMaxAccel() * 0.02, gains.kMaxAccel() * 0.02);
      volts =
          12 * pid.calculate(model.getAngularVelocityRPM() / 60, profiledVelocity)
              + gains.kV() * profiledVelocity
              + gains.kS() * Math.signum(profiledVelocity)
              + gains.kA() * (profiledVelocity - previous) / 0.02;
    }
    double battery = RobotController.getBatteryVoltage();
    volts = MathUtil.clamp(volts, -battery, battery);
    model.setInputVoltage(volts);
    model.update(0.02);
    inputs.position = model.getAngularPositionRotations();
    inputs.velocity = model.getAngularVelocityRPM() / 60;
    inputs.desiredVelocity = target;
    inputs.appliedVolts = volts;
    inputs.currentAmps = model.getCurrentDrawAmps();
    inputs.connected = true;
  }

  @Override
  public void setGains(FlywheelGains gains) {
    this.gains = gains;
    pid.setPID(gains.kP(), gains.kI(), gains.kD());
  }

  @Override
  public void setVelocity(double velocity) {
    if (!closedLoop) {
      pid.reset();
      profiledVelocity = model.getAngularVelocityRPM() / 60;
    }
    target = velocity;
    closedLoop = true;
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    target = 0;
    volts = voltage;
  }

  @Override
  public String getName() {
    return name;
  }
}
