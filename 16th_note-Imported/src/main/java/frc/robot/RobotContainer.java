package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.TankTeleopCommand;
import frc.robot.constants.MechanismConstants;
import frc.robot.constants.drive.TankDriveConstants;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.motor.Motor;
import frc.robot.subsystems.motor.MotorIO;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointIO;
import frc.robot.subsystems.tank.TankDrive;
import frc.robot.subsystems.tank.TankDriveIO;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Assembles mode-specific IO and commands following the 2026 base project pattern. */
public class RobotContainer {
  private final TankDrive drive = new TankDrive(TankDriveIO.fromSparkMax(MotorType.kBrushless));
  private final Flywheel shooter =
      new Flywheel(
          FlywheelIO.fromSparkMax("Shooter", ShooterConstants.FLYWHEEL_CONFIG),
          ShooterConstants.FLYWHEEL_GAINS);
  private final PositionJoint pivot =
      new PositionJoint(
          PositionJointIO.fromSparkMax(
              "Pivot", IntakeConstants.PIVOT_CONFIG, IntakeConstants.PIVOT_MAX_VOLTS),
          IntakeConstants.PIVOT_GAINS);
  private final Motor intake =
      new Motor("Intake", MotorIO.create(MechanismConstants.INTAKE_CAN_ID, 1, false, 0.002), 6);
  private final Motor indexer =
      new Motor("Indexer", MotorIO.create(MechanismConstants.INDEXER_CAN_ID, 1, false, 0.002), 6);
  private final LoggedTunableNumber stow =
      new LoggedTunableNumber(
          "Pivot/Presets/StowRotations", MechanismConstants.PIVOT_HOME_ROTATIONS);
  private final LoggedTunableNumber deployed =
      new LoggedTunableNumber(
          "Pivot/Presets/DeployedRotations", MechanismConstants.PIVOT_DEPLOYED_ROTATIONS);
  private final LoggedTunableNumber intermediate =
      new LoggedTunableNumber(
          "Pivot/Presets/IntermediateRotations", MechanismConstants.PIVOT_INTERMEDIATE_ROTATIONS);
  private final LoggedTunableNumber shooterA =
      new LoggedTunableNumber("Shooter/Presets/A_Rps", ShooterConstants.FREE_SPEED_RPS * 0.5);
  private final LoggedTunableNumber shooterX =
      new LoggedTunableNumber("Shooter/Presets/X_Rps", ShooterConstants.FREE_SPEED_RPS * 0.75);
  private final CommandXboxController driverController =
      new CommandXboxController(TankDriveConstants.DRIVER_CONTROLLER_PORT);
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  public void periodic() {
    autoChooser.periodic();
  }

  public RobotContainer() {
    // The active drivetrain is tank and uses manual controller input only.
    drive.setDefaultCommand(new TankTeleopCommand(drive, driverController));
    configureButtonBindings();
    autoChooser.addDefaultOption(
        "Do nothing", Commands.run(this::stop, drive, shooter, pivot, intake, indexer));
  }

  private void configureButtonBindings() {
    Trigger teleop =
        new Trigger(() -> DriverStation.isTeleopEnabled() && DriverStation.isJoystickConnected(0));
    intake.setDefaultCommand(
        intake.voltageCommand(
            () ->
                teleop.getAsBoolean()
                    ? 6
                        * (driverController.getRightTriggerAxis()
                            - driverController.getLeftTriggerAxis())
                    : 0));
    indexer.setDefaultCommand(
        indexer.voltageCommand(
            () ->
                teleop.getAsBoolean()
                    ? 6
                        * ((driverController.rightBumper().getAsBoolean() ? 1 : 0)
                            - (driverController.leftBumper().getAsBoolean() ? 1 : 0))
                    : 0));
    driverController.a().and(teleop).onTrue(Flywheel.holdVelocity(shooter, shooterA::get));
    driverController.x().and(teleop).onTrue(Flywheel.holdVelocity(shooter, shooterX::get));
    driverController.b().and(teleop).onTrue(shooter.stopCommand());
    driverController.povDown().and(teleop).onTrue(PositionJoint.holdPosition(pivot, stow::get));
    driverController.povUp().and(teleop).onTrue(PositionJoint.holdPosition(pivot, deployed::get));
    driverController
        .povRight()
        .and(teleop)
        .onTrue(PositionJoint.holdPosition(pivot, intermediate::get));
  }

  public void stop() {
    drive.stop();
    shooter.stop();
    pivot.stop();
    intake.stop();
    indexer.stop();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
