package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Locale;

/** Shared actions for Xbox, PS4, and Logitech Dual Action controllers. */
public final class DriverControls {
  public enum Layout {
    XBOX("Xbox"),
    PS4("PS4"),
    LOGITECH_DUAL_ACTION("Logitech Dual Action");

    private final String label;

    Layout(String label) {
      this.label = label;
    }
  }

  private final int port;
  private final boolean autoDetect;
  private final Layout fallback;
  private final GenericHID hid;
  private final XboxController xbox;
  private final PS4Controller ps4;

  public DriverControls(int port, boolean useXbox) {
    this(port, false, useXbox);
  }

  public DriverControls(int port, boolean autoDetect, boolean fallbackXbox) {
    this(port, autoDetect, fallbackXbox ? Layout.XBOX : Layout.PS4);
  }

  public DriverControls(int port, boolean autoDetect, Layout fallback) {
    this.port = port;
    this.autoDetect = autoDetect;
    this.fallback = java.util.Objects.requireNonNull(fallback);
    hid = new GenericHID(port);
    xbox = new XboxController(port);
    ps4 = new PS4Controller(port);
  }

  public Layout layout() {
    if (!autoDetect) return fallback;
    String name = DriverStation.getJoystickName(port).toLowerCase(Locale.ROOT);
    // XInput wrappers expose an Xbox mapping even for a physical Logitech gamepad.
    if (DriverStation.getJoystickIsXbox(port) || name.contains("xbox") || name.contains("xinput"))
      return Layout.XBOX;
    if (name.contains("dual action") || name.contains("dualaction")) {
      return Layout.LOGITECH_DUAL_ACTION;
    }
    // Preserve the existing native PS4 and unnamed simulator fallback behavior.
    return name.isBlank() ? fallback : Layout.PS4;
  }

  public String layoutName() {
    return layout().label + (autoDetect ? " (auto)" : " (manual)");
  }

  public boolean isConnected() {
    return DriverStation.isJoystickConnected(port);
  }

  public double getLeftY() {
    return switch (layout()) {
      case XBOX -> xbox.getLeftY();
      case PS4 -> ps4.getLeftY();
      case LOGITECH_DUAL_ACTION -> hid.getRawAxis(1);
    };
  }

  public double getRightX() {
    return switch (layout()) {
      case XBOX -> xbox.getRightX();
      case PS4 -> ps4.getRightX();
      case LOGITECH_DUAL_ACTION -> hid.getRawAxis(2);
    };
  }

  public boolean isVisionAssistRequested() {
    return switch (layout()) {
      case XBOX -> xbox.getRightTriggerAxis() > .5;
      case PS4 -> ps4.getR2Button();
      case LOGITECH_DUAL_ACTION -> hid.getRawButton(8);
    };
  }

  public boolean isIntakeRequested() {
    return switch (layout()) {
      case XBOX -> xbox.getLeftBumperButton();
      case PS4 -> ps4.getL1Button();
      case LOGITECH_DUAL_ACTION -> hid.getRawButton(5);
    };
  }

  public boolean isShootRequested() {
    return switch (layout()) {
      case XBOX -> xbox.getRightBumperButton();
      case PS4 -> ps4.getR1Button();
      case LOGITECH_DUAL_ACTION -> hid.getRawButton(6);
    };
  }

  /** Read-only diagnostics, called in all robot modes so checking axes needs no enabling. */
  public void publishDiagnostics() {
    boolean connected = isConnected();
    int count = connected ? hid.getAxisCount() : 0;
    SmartDashboard.putString("Drive/ControllerName", hid.getName());
    SmartDashboard.putString("Drive/ControllerLayout", layoutName());
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
