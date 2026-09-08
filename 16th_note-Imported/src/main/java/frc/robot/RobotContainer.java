package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.drive.TankDriveConstants;
import frc.robot.sim.replay.tank.TankDriveIOReplay;
import frc.robot.sim.tank.TankDriveIOSim;
import frc.robot.subsystems.tank.TankDrive;
import frc.robot.subsystems.tank.TankDriveIOPWM;
import frc.robot.vision.TankVisionDriveAdapter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.mecorobotics.gamepiecevision.GamePieceVisionClient;

/** Assembles mode-specific IO and commands following the 2026 base project pattern. */
public class RobotContainer {
  private final TankDrive drive =
      new TankDrive(
          switch (frc.robot.constants.Constants.currentMode) {
            case REAL -> new TankDriveIOPWM();
            case SIM -> new TankDriveIOSim();
            case REPLAY -> new TankDriveIOReplay();
          });
  private final TankVisionDriveAdapter visionDrive =
      new TankVisionDriveAdapter(
          drive,
          Constants.VisionPursuit.MAX_FORWARD_OUTPUT,
          Constants.VisionPursuit.MAX_TURN_OUTPUT);
  private final Joystick leftStick = new Joystick(TankDriveConstants.LEFT_JOYSTICK_PORT);
  private final Joystick rightStick = new Joystick(TankDriveConstants.RIGHT_JOYSTICK_PORT);
  private final GamePieceVisionClient vision =
      new GamePieceVisionClient(Constants.VisionPursuit.CAMERA_NAME);
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  public RobotContainer() {
    drive.setDefaultCommand(
        Commands.runEnd(this::driveTeleop, drive::stop, drive)
            .withName("Tank teleop with vision assist"));
    // PWM-only hardware has no measured wheel position or gyro for path following.
    autoChooser.addDefaultOption("Do nothing", Commands.run(drive::stop, drive));
  }

  private void driveTeleop() {
    if (!DriverStation.isTeleopEnabled()) {
      drive.stop();
      return;
    }
    double left = -leftStick.getY();
    double right = -rightStick.getY();
    boolean manual =
        Math.abs(left) > Constants.VisionPursuit.MANUAL_OVERRIDE_DEADBAND
            || Math.abs(right) > Constants.VisionPursuit.MANUAL_OVERRIDE_DEADBAND;
    var request = vision.getRequest(rightStick.getTrigger(), manual);
    if (request.active()) {
      if (request.atGoal()) {
        drive.stop();
      } else {
        drive.arcadeDrive(
            request.forward() * Constants.VisionPursuit.MAX_FORWARD_OUTPUT,
            request.turn() * Constants.VisionPursuit.MAX_TURN_OUTPUT);
      }
    } else {
      drive.tankDrive(left, right);
    }
  }

  public void stop() {
    drive.stop();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
