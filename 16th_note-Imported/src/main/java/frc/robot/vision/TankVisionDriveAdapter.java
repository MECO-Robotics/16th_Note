package frc.robot.vision;

import frc.robot.subsystems.tank.TankDrive;
import org.mecorobotics.gamepiecevision.GamePieceVisionClient.DriveRequest;

public final class TankVisionDriveAdapter implements VisionDriveAdapter {
  private final TankDrive drivetrain;
  private final double maxForward;
  private final double maxTurn;

  public TankVisionDriveAdapter(TankDrive drivetrain, double maxForward, double maxTurn) {
    this.drivetrain = drivetrain;
    this.maxForward = maxForward;
    this.maxTurn = maxTurn;
  }

  @Override
  public void apply(DriveRequest request) {
    if (!request.active() || request.atGoal()) {
      stop();
      return;
    }
    drivetrain.arcadeDrive(request.forward() * maxForward, request.turn() * maxTurn);
  }

  @Override
  public void stop() {
    drivetrain.stop();
  }
}
