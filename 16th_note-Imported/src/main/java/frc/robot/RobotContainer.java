package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.TankTeleopCommand;
import frc.robot.commands.fuel.PreloadAutoCommand;
import frc.robot.constants.drive.TankDriveConstants;
import frc.robot.controls.DriverControls;
import frc.robot.subsystems.fuel.FuelSystem;
import frc.robot.subsystems.tank.TankDrive;
import frc.robot.subsystems.tank.TankDriveIO;
import frc.robot.vision.TankVisionDriveAdapter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.mecorobotics.gamepiecevision.GamePieceVisionClient;

/** Assembles mode-specific IO and commands following the 2026 base project pattern. */
public class RobotContainer {
  private final FuelSystem fuel = FuelSystem.create();
  private final TankDrive drive = new TankDrive(TankDriveIO.fromSparkMax(MotorType.kBrushless));
  private final TankVisionDriveAdapter visionDrive =
      new TankVisionDriveAdapter(
          drive,
          Constants.VisionPursuit.MAX_FORWARD_OUTPUT,
          Constants.VisionPursuit.MAX_TURN_OUTPUT);
  private final DriverControls driverController =
      new DriverControls(
          TankDriveConstants.DRIVER_CONTROLLER_PORT,
          TankDriveConstants.AUTO_DETECT_CONTROLLER,
          TankDriveConstants.CONTROLLER_LAYOUT);
  private final GamePieceVisionClient vision =
      new GamePieceVisionClient(Constants.VisionPursuit.CAMERA_NAME);
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  private final frc.robot.sim.fuel.FuelPracticeSimulation practice =
      frc.robot.constants.Constants.currentMode == frc.robot.constants.Constants.Mode.SIM
          ? new frc.robot.sim.fuel.FuelPracticeSimulation(drive, fuel)
          : null;

  public void publishControllerDiagnostics() {
    driverController.publishDiagnostics();
  }

  public void simulationPeriodic() {
    if (practice != null) practice.periodic();
  }

  public RobotContainer() {
    fuel.setDefaultCommand(fuel.teleopCommand(driverController));
    drive.setDefaultCommand(new TankTeleopCommand(drive, driverController, vision));
    autoChooser.addOption("Back up and shoot preloads", new PreloadAutoCommand(drive, fuel));
    // Wheel distance and heading are not calibrated for path following.
    autoChooser.addDefaultOption("Do nothing", Commands.run(drive::stop, drive));
  }

  public void startCommissioning() {
    CommandScheduler.getInstance().schedule(fuel.commissioningCommand(driverController, drive));
  }

  public void stop() {
    drive.stop();
    fuel.stop();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
