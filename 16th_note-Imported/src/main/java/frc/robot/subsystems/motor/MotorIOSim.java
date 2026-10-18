package frc.robot.subsystems.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Open-loop roller plant; transmission inertia is an estimate. */
public final class MotorIOSim implements MotorIO {
  private final DCMotorSim model;
  private double volts;

  public MotorIOSim(double reduction, double inertia) {
    var motor = DCMotor.getNEO(1);
    model = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, inertia, reduction), motor);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    model.setInputVoltage(volts);
    model.update(0.02);
    inputs.positionRotations = model.getAngularPositionRotations();
    inputs.velocityRpm = model.getAngularVelocityRPM();
    inputs.appliedVolts = volts;
    inputs.currentAmps = model.getCurrentDrawAmps();
    inputs.connected = true;
  }

  @Override
  public void setVoltage(double value) {
    double battery = RobotController.getBatteryVoltage();
    volts = MathUtil.clamp(value, -battery, battery);
  }
}
