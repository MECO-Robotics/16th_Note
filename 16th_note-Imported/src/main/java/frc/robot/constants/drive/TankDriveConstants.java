package frc.robot.constants.drive;

/** Wiring from the original PWM tank robot; verify inversion before driving. */
public final class TankDriveConstants {
  public static final int LEFT_PWM_PORT = 0;
  public static final int RIGHT_PWM_PORT = 1;
  public static final boolean LEFT_INVERTED = false;
  public static final boolean RIGHT_INVERTED = true;
  public static final int LEFT_JOYSTICK_PORT = 0;
  public static final int RIGHT_JOYSTICK_PORT = 1;
  public static final double DEADBAND = 0.02;
  public static final double MAX_VOLTS = 12.0;
  // Simulation estimates only. Measure the robot before using model-based autonomous.
  public static final double SIM_GEAR_RATIO = 10.71;
  public static final double SIM_MASS_KG = 50.0;
  public static final double SIM_MOI_KG_METERS_SQUARED = 6.0;
  public static final double SIM_WHEEL_RADIUS_METERS = 0.0762;
  public static final double SIM_TRACK_WIDTH_METERS = 0.6;

  private TankDriveConstants() {}
}
