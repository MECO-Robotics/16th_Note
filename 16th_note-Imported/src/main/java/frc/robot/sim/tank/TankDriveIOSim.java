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
  private edu.wpi.first.math.geometry.Pose2d safePose = frc.robot.sim.fuel.FuelWorldSim.START;
  private double leftVolts;
  private double rightVolts;

  public TankDriveIOSim() {
    model.setPose(frc.robot.sim.fuel.FuelWorldSim.START);
  }

  @Override
  public void resetSimulationPose(edu.wpi.first.math.geometry.Pose2d pose) {
    leftVolts = 0;
    rightVolts = 0;
    model.setState(edu.wpi.first.math.VecBuilder.fill(0, 0, 0, 0, 0, 0, 0));
    model.setPose(pose);
    safePose = pose;
  }

  @Override
  public void updateInputs(TankDriveIOInputs inputs) {
    var previous = safePose;
    model.setInputs(leftVolts, rightVolts);
    model.update(0.02);
    var requested = model.getPose();
    var constrained = PracticeField.constrain(previous, requested);
    if (constrained.getTranslation().getDistance(requested.getTranslation()) > 1e-9) {
      model.setState(
          edu.wpi.first.math.VecBuilder.fill(
              constrained.getX(),
              constrained.getY(),
              constrained.getRotation().getRadians(),
              0,
              0,
              model.getLeftPositionMeters(),
              model.getRightPositionMeters()));
    }
    safePose = constrained;
    inputs.leftAppliedVolts = leftVolts;
    inputs.rightAppliedVolts = rightVolts;
    inputs.hasPositionFeedback = true;
    inputs.leftPositionMeters = model.getLeftPositionMeters();
    inputs.rightPositionMeters = model.getRightPositionMeters();
    inputs.leftVelocityMetersPerSecond = model.getLeftVelocityMetersPerSecond();
    inputs.rightVelocityMetersPerSecond = model.getRightVelocityMetersPerSecond();
    inputs.headingRadians = safePose.getRotation().getRadians();
    inputs.simulatedPose = safePose;
  }

  @Override
  public void setVoltage(double left, double right) {
    double battery = RobotController.getBatteryVoltage();
    leftVolts = MathUtil.clamp(left, -battery, battery);
    rightVolts = MathUtil.clamp(right, -battery, battery);
  }
}
