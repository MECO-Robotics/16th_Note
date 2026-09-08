package frc.robot;

/** Robot-specific vision settings. Runtime mode lives in frc.robot.constants.Constants. */
public final class Constants {
  public static final class VisionPursuit {
    public static final String CAMERA_NAME = "left";
    public static final double MAX_FORWARD_OUTPUT = 0.35;
    public static final double MAX_TURN_OUTPUT = 0.35;
    public static final double MANUAL_OVERRIDE_DEADBAND = 0.15;

    private VisionPursuit() {}
  }

  private Constants() {}
}
