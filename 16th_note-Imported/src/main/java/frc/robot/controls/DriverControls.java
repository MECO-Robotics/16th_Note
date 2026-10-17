package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Xbox controller actions on the configured Driver Station USB port. */
public final class DriverControls {
  private final int port;
  private final GenericHID hid;
  private final XboxController xbox;

  public DriverControls(int port) {
    this.port = port;
    hid = new GenericHID(port);
    xbox = new XboxController(port);
  }

  public boolean isConnected() {
    return DriverStation.isJoystickConnected(port);
  }

  public double getLeftY() {
    return xbox.getLeftY();
  }

  public double getRightX() {
    return xbox.getRightX();
  }

  public boolean isVisionAssistRequested() {
    return xbox.getRightTriggerAxis() > .5;
  }

  public boolean isIntakeRequested() {
    return xbox.getLeftBumperButton();
  }

  public boolean isShootRequested() {
    return xbox.getRightBumperButton();
  }

  /** Read-only diagnostics, called in all robot modes so checking axes needs no enabling. */
  public void publishDiagnostics() {
    boolean connected = isConnected();
    int count = connected ? hid.getAxisCount() : 0;
    SmartDashboard.putString("Drive/ControllerName", hid.getName());
    SmartDashboard.putString("Drive/ControllerLayout", "Xbox");
    SmartDashboard.putBoolean("Drive/ControllerConnected", connected);
    SmartDashboard.putBoolean("Drive/ControllerIsXbox", DriverStation.getJoystickIsXbox(port));
    SmartDashboard.putNumber("Drive/ControllerPort", port);
    SmartDashboard.putNumber("Drive/ControllerAxisCount", count);
    SmartDashboard.putNumber("Drive/ControllerButtonCount", connected ? hid.getButtonCount() : 0);
    for (int axis = 0; axis < 6; axis++) {
      SmartDashboard.putNumber("Drive/RawAxis" + axis, axis < count ? hid.getRawAxis(axis) : 0);
    }
  }
}
