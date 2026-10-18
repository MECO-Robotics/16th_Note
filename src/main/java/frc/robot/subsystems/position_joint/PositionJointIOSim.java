package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;

/** Estimated joint plant; load inertia is provisional and gravity is not simulated. */
public final class PositionJointIOSim implements PositionJointIO {
  private final DCMotorSim model;
  private final String name;
  private final double homePosition, maxVolts;
  private double minimumBound = Double.NaN, maximumBound = Double.NaN;
  private final ProfiledPIDController pid =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0.25, 0.5));
  private PositionJointGains gains;
  private double volts;
  private boolean controlling;
  private double target;

  public PositionJointIOSim(String name, PositionJointHardwareConfig config, double maxVolts) {
    this.name = name;
    this.maxVolts = maxVolts;
    homePosition = config.encoderOffset().getRotations();
    double outputInertia =
        config.momentOfInertiaKgMetersSquared() * config.gearRatio() * config.gearRatio();
    var motor = DCMotor.getNEO(1);
    model =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, outputInertia, config.gearRatio()), motor);
    target = homePosition;
    resetPosition();
  }

  @Override
  public void updateInputs(PositionJointIOInputs inputs) {
    double position = model.getAngularPositionRotations();
    if (controlling && gains != null) {
      double previousVelocity = pid.getSetpoint().velocity;
      double pidVolts = 12 * pid.calculate(position, target);
      var setpoint = pid.getSetpoint();
      volts =
          MathUtil.clamp(
              pidVolts
                  + gains.kG() * Math.cos(2 * Math.PI * position)
                  + gains.kS() * Math.signum(setpoint.velocity)
                  + gains.kV() * setpoint.velocity
                  + gains.kA() * (setpoint.velocity - previousVelocity) / 0.02,
              -maxVolts,
              maxVolts);
      if ((position <= gains.kMinPosition() && volts < 0)
          || (position >= gains.kMaxPosition() && volts > 0)) volts = 0;
    }
    model.setInputVoltage(volts);
    model.update(0.02);
    double constrained =
        Double.isFinite(minimumBound)
            ? MathUtil.clamp(model.getAngularPositionRotations(), minimumBound, maximumBound)
            : model.getAngularPositionRotations();
    if (constrained != model.getAngularPositionRotations())
      model.setState(constrained * 2 * Math.PI, 0);
    inputs.outputPosition = model.getAngularPositionRotations();
    inputs.velocity = model.getAngularVelocityRadPerSec() / (2 * Math.PI);
    inputs.appliedVolts = volts;
    inputs.currentAmps = model.getCurrentDrawAmps();
    inputs.connected = inputs.referenced = true;
    inputs.configurationStatus = "Simulation";
  }

  @Override
  public void setGains(PositionJointGains gains) {
    this.gains = gains;
    if (!Double.isFinite(minimumBound)) {
      minimumBound = gains.kMinPosition();
      maximumBound = gains.kMaxPosition();
    }
    pid.setPID(gains.kP(), gains.kI(), gains.kD());
    pid.setConstraints(new TrapezoidProfile.Constraints(gains.kMaxVelo(), gains.kMaxAccel()));
  }

  @Override
  public void setPosition(double position, double velocity) {
    if (!controlling)
      pid.reset(
          model.getAngularPositionRotations(), model.getAngularVelocityRadPerSec() / (2 * Math.PI));
    target = MathUtil.clamp(position, gains.kMinPosition(), gains.kMaxPosition());
    controlling = true;
  }

  @Override
  public void setVoltage(double value) {
    controlling = false;
    volts = 0;
  }

  @Override
  public void resetPosition() {
    model.setState(homePosition * 2 * Math.PI, 0);
    pid.reset(homePosition);
    controlling = false;
    volts = 0;
  }

  @Override
  public String getName() {
    return name;
  }
}
