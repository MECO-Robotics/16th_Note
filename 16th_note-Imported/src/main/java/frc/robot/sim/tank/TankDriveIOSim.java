package frc.robot.sim.tank;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.constants.drive.TankDriveConstants;
import frc.robot.subsystems.tank.TankDriveIO;

/** Two-sided drivetrain physics with estimated dimensions; no real hardware is allocated. */
public final class TankDriveIOSim implements TankDriveIO {
  private final DifferentialDrivetrainSim model =
      new DifferentialDrivetrainSim(
          DCMotor.getNEO(1),
          TankDriveConstants.SIM_GEAR_RATIO,
          TankDriveConstants.SIM_MOI_KG_METERS_SQUARED,
          TankDriveConstants.SIM_MASS_KG,
          TankDriveConstants.SIM_WHEEL_RADIUS_METERS,
          TankDriveConstants.SIM_TRACK_WIDTH_METERS,
          null);
  private double leftVolts;
  private double rightVolts;

  @Override
  public void updateInputs(TankDriveIOInputs inputs) {
    model.setInputs(leftVolts, rightVolts);
    model.update(0.02);
    inputs.leftAppliedVolts = leftVolts;
    inputs.rightAppliedVolts = rightVolts;
    inputs.hasPositionFeedback = true;
    inputs.leftPositionMeters = model.getLeftPositionMeters();
    inputs.rightPositionMeters = model.getRightPositionMeters();
    inputs.leftVelocityMetersPerSecond = model.getLeftVelocityMetersPerSecond();
    inputs.rightVelocityMetersPerSecond = model.getRightVelocityMetersPerSecond();
    inputs.headingRadians = model.getHeading().getRadians();
  }

  @Override
  public void setVoltage(double left, double right) {
    double battery = RobotController.getBatteryVoltage();
    leftVolts = MathUtil.clamp(left, -battery, battery);
    rightVolts = MathUtil.clamp(right, -battery, battery);
  }
}
