package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Shared driver actions with an explicitly selected controller layout. */
public final class DriverControls {
  private final int port;
  private final DoubleSupplier leftY;
  private final DoubleSupplier rightX;
  private final BooleanSupplier visionAssist;
  private final BooleanSupplier intake;
  private final BooleanSupplier shoot;

  public DriverControls(int port, boolean useXbox) {
    this.port = port;
    if (useXbox) {
      XboxController controller = new XboxController(port);
      leftY = controller::getLeftY;
      rightX = controller::getRightX;
      visionAssist = () -> controller.getRightTriggerAxis() > 0.5;
      intake = controller::getLeftBumperButton;
      shoot = controller::getRightBumperButton;
    } else {
      PS4Controller controller = new PS4Controller(port);
      leftY = controller::getLeftY;
      rightX = controller::getRightX;
      visionAssist = controller::getR2Button;
      intake = controller::getL1Button;
      shoot = controller::getR1Button;
    }
  }

  public boolean isConnected() {
    return DriverStation.isJoystickConnected(port);
  }

  public double getLeftY() {
    return leftY.getAsDouble();
  }

  public double getRightX() {
    return rightX.getAsDouble();
  }

  public boolean isVisionAssistRequested() {
    return visionAssist.getAsBoolean();
  }

  public boolean isIntakeRequested() {
    return intake.getAsBoolean();
  }

  public boolean isShootRequested() {
    return shoot.getAsBoolean();
  }
}
