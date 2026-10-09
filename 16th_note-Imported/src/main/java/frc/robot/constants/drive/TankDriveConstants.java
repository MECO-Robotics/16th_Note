package frc.robot.constants.drive;

/** CAN drivetrain wiring; verify inversion before driving. */
public final class TankDriveConstants {
  public static final int LEFT_CAN_ID = 1;
  public static final int RIGHT_CAN_ID = 2;
  public static final boolean LEFT_INVERTED = false;
  public static final boolean RIGHT_INVERTED = true;
  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final boolean AUTO_DETECT_CONTROLLER = true;
  // Used for unnamed simulation devices or when AUTO_DETECT_CONTROLLER is false.
  // Options: XBOX, PS4, LOGITECH_DUAL_ACTION.
  public static final frc.robot.controls.DriverControls.Layout CONTROLLER_LAYOUT =
      frc.robot.controls.DriverControls.Layout.XBOX;
  public static final double DEADBAND = 0.02;
  public static final double MAX_VOLTS = 12.0;
  // Owner-reported MAXPlanetary 4:1 gearbox, with 22T output driving 22T wheel sprockets.
  public static final double SIM_GEAR_RATIO = 4.0;
  // Owner-reported 3-inch drive-wheel diameter (1.5-inch radius).
  public static final double SIM_WHEEL_RADIUS_METERS = 0.0381;
  // Remaining simulation estimates. Measure before using model-based autonomous.
  public static final double SIM_MASS_KG = 50.0;
  public static final double SIM_MOI_KG_METERS_SQUARED = 6.0;
  // Owner-reported left-to-right wheel-center spacing: 9.414 inches.
  public static final double SIM_TRACK_WIDTH_METERS = 9.414 * 0.0254;

  private TankDriveConstants() {}
}
