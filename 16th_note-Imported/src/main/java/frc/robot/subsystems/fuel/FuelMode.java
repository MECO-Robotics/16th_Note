package frc.robot.subsystems.fuel;

import edu.wpi.first.wpilibj.DriverStation;

public enum FuelMode {
  DISABLED,
  TELEOP,
  AUTONOMOUS,
  TEST;

  public static FuelMode current() {
    if (DriverStation.isDisabled()) return DISABLED;
    if (DriverStation.isAutonomousEnabled()) return AUTONOMOUS;
    if (DriverStation.isTestEnabled()) return TEST;
    return TELEOP;
  }
}
