package frc.robot.constants.vision;

/** Distance-based shooter defaults. Calibration remains disabled until it is measured on Pete. */
public final class VisionConstants {
  public static final double MAX_OBSERVATION_AGE_SECONDS = 0.25;
  public static final double MAX_POSE_AMBIGUITY = 0.2;
  public static final double LOST_TARGET_HOLD_SECONDS = 0.15;

  private VisionConstants() {}
}
