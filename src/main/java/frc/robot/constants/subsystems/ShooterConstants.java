package frc.robot.constants.subsystems;

import static frc.robot.constants.MechanismConstants.*;

import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;

public final class ShooterConstants {
  public static final FlywheelHardwareConfig FLYWHEEL_CONFIG =
      new FlywheelHardwareConfig(
          new int[] {SHOOTER_CAN_ID},
          new boolean[] {SHOOTER_INVERTED},
          SHOOTER_GEAR_RATIO,
          0.003,
          60,
          "");
  public static final double FREE_SPEED_RPS = NEO_FREE_SPEED_RPM / (60 * SHOOTER_GEAR_RATIO);
  // RPS units, matching the rebuilt flywheel IO. kP is controller duty/RPS; kV is V/RPS.
  public static final FlywheelGains FLYWHEEL_GAINS =
      new FlywheelGains(0.001 * 60 / 12, 0, 0, 0, 12 / FREE_SPEED_RPS, 0, 10000.0 / 60, 150.0 / 60);

  private ShooterConstants() {}
}
