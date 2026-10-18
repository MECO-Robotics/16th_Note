package frc.robot.constants.subsystems;

import static frc.robot.constants.MechanismConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.types.PositionJointConstants.EncoderType;
import frc.robot.constants.types.PositionJointConstants.MechanismType;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;

public final class IntakeConstants {
  public static final PositionJointHardwareConfig PIVOT_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {PIVOT_CAN_ID},
          new boolean[] {PIVOT_INVERTED},
          PIVOT_GEAR_RATIO,
          0.05 / (PIVOT_GEAR_RATIO * PIVOT_GEAR_RATIO),
          60,
          EncoderType.INTERNAL,
          0,
          MechanismType.ROTATIONAL,
          0,
          Rotation2d.fromRotations(PIVOT_HOME_ROTATIONS),
          "");
  public static final double PIVOT_MAX_VOLTS = 3;
  public static final PositionJointGains PIVOT_GAINS =
      new PositionJointGains(
          10.0 / 12.0,
          0,
          0,
          0,
          0,
          0,
          0,
          0.25,
          0.5,
          PIVOT_MIN_POSITION,
          PIVOT_MAX_POSITION,
          0.01,
          PIVOT_HOME_ROTATIONS);

  private IntakeConstants() {}
}
