package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.tank.TankDrive;

/** Default manual arcade command for the tank drivetrain. */
public class TankTeleopCommand extends Command {
  private final TankDrive drive;
  private final CommandXboxController controller;

  public TankTeleopCommand(TankDrive drive, CommandXboxController controller) {
    this.drive = drive;
    this.controller = controller;
    addRequirements(drive);
    setName("Manual arcade teleop");
  }

  @Override
  public void execute() {
    if (!DriverStation.isTeleopEnabled()
        || !DriverStation.isJoystickConnected(controller.getHID().getPort())) {
      drive.stop();
      return;
    }
    double forward = -controller.getLeftY();
    double turn = -controller.getRightX();
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/ForwardInput", forward);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Drive/TurnInput", turn);
    drive.arcadeDriveManual(forward, turn);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
