package frc.robot.constants;

/** Reductions are motor rotations per mechanism rotation. */
public final class MechanismConstants {
  public static final int INDEXER_CAN_ID = 3;
  public static final int PIVOT_CAN_ID = 5;
  public static final boolean PIVOT_INVERTED = false;
  public static final int INTAKE_CAN_ID = 6;
  public static final int SHOOTER_CAN_ID = 7;
  public static final boolean SHOOTER_INVERTED = true;
  public static final int SHOOTER_PINION_TEETH = 22;
  public static final int SHOOTER_OUTPUT_TEETH = 26;
  public static final double SHOOTER_GEAR_RATIO =
      (double) SHOOTER_OUTPUT_TEETH / SHOOTER_PINION_TEETH;
  public static final int PIVOT_PINION_GEAR_TEETH = 22;
  public static final int PIVOT_DRIVEN_GEAR_TEETH = 38;
  public static final int PIVOT_INPUT_SPROCKET_TEETH = 12;
  public static final int PIVOT_OUTPUT_SPROCKET_TEETH = 54;
  // Motor pinion -> driven gear and attached input sprocket -> pivot output sprocket.
  public static final double PIVOT_GEAR_RATIO =
      ((double) PIVOT_DRIVEN_GEAR_TEETH / PIVOT_PINION_GEAR_TEETH)
          * ((double) PIVOT_OUTPUT_SPROCKET_TEETH / PIVOT_INPUT_SPROCKET_TEETH);
  // Separate intake/indexer transmission ratios have not been measured.
  // Their open-loop controls do not use mechanism position or velocity.
  public static final double NEO_FREE_SPEED_RPM = 5676.0;
  public static final double PIVOT_HOME_ROTATIONS = 90.0 / 360.0;
  public static final double PIVOT_DEPLOYED_ROTATIONS = -60.0 / 360.0;
  public static final double PIVOT_MAX_ROTATIONS = PIVOT_HOME_ROTATIONS - PIVOT_DEPLOYED_ROTATIONS;
  public static final double PIVOT_INTERMEDIATE_ROTATIONS =
      (PIVOT_HOME_ROTATIONS + PIVOT_DEPLOYED_ROTATIONS) / 2;
  public static final double PIVOT_MIN_POSITION = PIVOT_DEPLOYED_ROTATIONS;
  public static final double PIVOT_MAX_POSITION = PIVOT_HOME_ROTATIONS;

  private MechanismConstants() {}
}
