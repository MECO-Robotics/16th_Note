package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.PreloadAutoCommands;
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
  private final LoggedTunableNumber autoSpinupSeconds =
      new LoggedTunableNumber("Auto/Shoot/SpinupSeconds", 1.5);
  private final LoggedTunableNumber autoFullSpeedRps =
      new LoggedTunableNumber("Auto/Shoot/FullSpeed_Rps", ShooterConstants.FREE_SPEED_RPS);
  private final LoggedTunableNumber autoBackupSpeedRps =
      new LoggedTunableNumber("Auto/Shoot/BackupSpeed_Rps", ShooterConstants.FREE_SPEED_RPS * 0.75);
  private final LoggedTunableNumber autoReadyTimeoutSeconds =
      new LoggedTunableNumber("Auto/Shoot/ReadyTimeoutSeconds", 5.0);
  private final LoggedTunableNumber autoFeedSeconds =
      new LoggedTunableNumber("Auto/Shoot/FeedSeconds", 1.5);
  private final LoggedTunableNumber autoIndexerVolts =
      new LoggedTunableNumber("Auto/Shoot/IndexerVolts", 6.0);
  private final LoggedTunableNumber autoIntakeVolts =
      new LoggedTunableNumber("Auto/Intake/Volts", 6.0);
  private final LoggedTunableNumber autoPivotTimeoutSeconds =
      new LoggedTunableNumber("Auto/Pivot/TimeoutSeconds", 2.0);
  private final CommandXboxController driverController =
      new CommandXboxController(TankDriveConstants.DRIVER_CONTROLLER_PORT);
  private final LoggedDashboardChooser<Command> autoChooser;

  public void periodic() {
    autoChooser.periodic();
  }

  public RobotContainer() {
    drive.setDefaultCommand(new TankTeleopCommand(drive, driverController));
    configureButtonBindings();
    registerNamedCommands();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "DeployIntake",
        PreloadAutoCommands.moveIntake(
            pivot, deployed::get, autoPivotTimeoutSeconds::get, "DeployIntake"));
    NamedCommands.registerCommand(
        "RaiseIntakeToFeed",
        PreloadAutoCommands.moveIntake(
            pivot, intermediate::get, autoPivotTimeoutSeconds::get, "RaiseIntakeToFeed"));
    NamedCommands.registerCommand(
        "StowIntake",
        PreloadAutoCommands.moveIntake(
            pivot, stow::get, autoPivotTimeoutSeconds::get, "StowIntake"));
    NamedCommands.registerCommand(
        "SpinIntake", intake.voltageCommand(autoIntakeVolts::get).withName("SpinIntake"));
    NamedCommands.registerCommand(
        "StopIntake", Commands.runOnce(intake::stop, intake).withName("StopIntake"));
    NamedCommands.registerCommand(
        "SpinUpShooter", Flywheel.holdVelocity(shooter, shooterA::get).withName("SpinUpShooter"));
    NamedCommands.registerCommand(
        "StopShooter", Commands.runOnce(shooter::stop, shooter).withName("StopShooter"));
    NamedCommands.registerCommand(
        "FeedIndexer", indexer.voltageCommand(autoIndexerVolts::get).withName("FeedIndexer"));
    NamedCommands.registerCommand(
        "FeedRollers", indexer.voltageCommand(autoIndexerVolts::get).withName("FeedRollers"));
    NamedCommands.registerCommand(
        "StopIndexer", Commands.runOnce(indexer::stop, indexer).withName("StopIndexer"));
    NamedCommands.registerCommand(
        "IdleRollers",
        Commands.runOnce(
                () -> {
                  intake.stop();
                  indexer.stop();
                },
                intake,
                indexer)
            .withName("IdleRollers"));
    NamedCommands.registerCommand(
        "Agitate", PositionJoint.holdPosition(pivot, intermediate::get).withName("Agitate"));
    NamedCommands.registerCommand(
        "AutoSpinUp",
        Commands.run(() -> shooter.setVelocity(shooterA.get()), shooter)
            .withTimeout(2.0)
            .withName("AutoSpinUp"));
    NamedCommands.registerCommand("Shoot", shootCommand());
    NamedCommands.registerCommand("Shoot75Percent", shoot75PercentCommand());
    NamedCommands.registerCommand("ShootFullSpeed", shootFullSpeedCommand());
    NamedCommands.registerCommand(
        "StopAll",
        Commands.runOnce(this::stop, drive, shooter, pivot, intake, indexer).withName("StopAll"));
  }

  private Command shootCommand() {
    return PreloadAutoCommands.shootPreloads(
        shooter,
        pivot,
        indexer,
        shooterA::get,
        intermediate::get,
        deployed::get,
        autoSpinupSeconds::get,
        autoFeedSeconds::get,
        autoIndexerVolts::get,
        autoPivotTimeoutSeconds::get);
  }

  private Command shootFullSpeedCommand() {
    return PreloadAutoCommands.shootWhenReady(
        shooter,
        pivot,
        indexer,
        autoFullSpeedRps::get,
        intermediate::get,
        autoReadyTimeoutSeconds::get,
        autoFeedSeconds::get,
        autoIndexerVolts::get,
        "ShootFullSpeed");
  }

  private Command shoot75PercentCommand() {
    return PreloadAutoCommands.shootWhenReady(
        shooter,
        pivot,
        indexer,
        autoBackupSpeedRps::get,
        intermediate::get,
        autoReadyTimeoutSeconds::get,
        autoFeedSeconds::get,
        autoIndexerVolts::get,
        "Shoot75Percent");
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
