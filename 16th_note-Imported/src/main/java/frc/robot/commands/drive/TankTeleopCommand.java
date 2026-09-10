package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.tank.TankDrive;
import org.mecorobotics.gamepiecevision.GamePieceVisionClient;

/** Arcade driving with PS4 sticks and R2-held vision assist. */
public class TankTeleopCommand extends Command {
  private final TankDrive drive;
  private final PS4Controller controller;
  private final GamePieceVisionClient vision;

  public TankTeleopCommand(
      TankDrive drive, PS4Controller controller, GamePieceVisionClient vision) {
    this.drive = drive;
    this.controller = controller;
    this.vision = vision;
    addRequirements(drive);
    setName("Arcade teleop with vision assist");
  }

  @Override
  public void execute() {
    if (!DriverStation.isTeleopEnabled()) {
      drive.stop();
      return;
    }
    double forward = -controller.getLeftY();
    double turn = -controller.getRightX();
    boolean manual =
        Math.abs(forward) > Constants.VisionPursuit.MANUAL_OVERRIDE_DEADBAND
            || Math.abs(turn) > Constants.VisionPursuit.MANUAL_OVERRIDE_DEADBAND;
    var request = vision.getRequest(controller.getR2Button(), manual);
    if (request.active()) {
      if (request.atGoal()) {
        drive.stop();
      } else {
        drive.arcadeDrive(
            request.forward() * Constants.VisionPursuit.MAX_FORWARD_OUTPUT,
            request.turn() * Constants.VisionPursuit.MAX_TURN_OUTPUT);
      }
    } else {
      drive.arcadeDriveManual(forward, turn);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
