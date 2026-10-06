package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Shared driver actions with automatic Xbox/PS4 detection or a manual layout override. */
public final class DriverControls {
  private final int port;
  private final DoubleSupplier leftY;
  private final DoubleSupplier rightX;
  private final BooleanSupplier visionAssist;
  private final BooleanSupplier intake;
  private final BooleanSupplier shoot;

  private final boolean autoDetect;
  private final boolean fallbackXbox;

  public DriverControls(int port, boolean useXbox) {
    this(port, false, useXbox);
  }

  public DriverControls(int port, boolean autoDetect, boolean fallbackXbox) {
    this.port = port;
    this.autoDetect = autoDetect;
    this.fallbackXbox = fallbackXbox;
    XboxController xbox = new XboxController(port);
    PS4Controller ps4 = new PS4Controller(port);
    leftY = () -> usesXbox() ? xbox.getLeftY() : ps4.getLeftY();
    rightX = () -> usesXbox() ? xbox.getRightX() : ps4.getRightX();
    visionAssist = () -> usesXbox() ? xbox.getRightTriggerAxis() > .5 : ps4.getR2Button();
    intake = () -> usesXbox() ? xbox.getLeftBumperButton() : ps4.getL1Button();
    shoot = () -> usesXbox() ? xbox.getRightBumperButton() : ps4.getR1Button();
  }

  private boolean usesXbox() {
    if (!autoDetect) return fallbackXbox;
    if (DriverStation.getJoystickIsXbox(port)) return true;
    String name = DriverStation.getJoystickName(port).toLowerCase(java.util.Locale.ROOT);
    if (name.contains("xbox") || name.contains("xinput")) return true;
    // Native PS4 controllers commonly identify as "Wireless Controller".
    // These are the two supported layouts; unnamed simulation devices use the fallback.
    return name.isBlank() ? fallbackXbox : false;
  }

  public String layoutName() {
    return (usesXbox() ? "Xbox" : "PS4") + (autoDetect ? " (auto)" : " (manual)");
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
