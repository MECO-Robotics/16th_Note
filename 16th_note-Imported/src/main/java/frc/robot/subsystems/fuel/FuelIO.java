package frc.robot.subsystems.fuel;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware boundary for the coordinated fuel mechanism. Real IO must validate configuration,
 * refresh all required signals, invalidate the position reference after a controller reset, and
 * enforce current, voltage, travel and motion limits independently of the sequence.
 */
public interface FuelIO {
  @AutoLog
  class FuelIOInputs {
    public boolean available = false;
    public boolean[] configured = new boolean[5];
    public boolean[] connected = new boolean[5];
    public double[] motorRotations = new double[5];
    public double[] motorRpm = new double[5];
    public double[] currentAmps = new double[5];
    public double[] appliedVolts = new double[5];
    public double[] temperatureCelsius = new double[5];
    public boolean[] resetDetected = new boolean[5];
    public boolean[] brownoutDetected = new boolean[5];
    public int[] controllerFaults = new int[5];
    public String configurationStatus = "Unconfigured";
    public String referenceStatus = "Not referenced";
    public String requestMode = "NONE";
    // Health comes from controller read status; age is time since a successful read, not CAN-frame
    // age.
    public boolean healthy = false;
    public boolean shooterPrimaryConnected = false;
    public boolean shooterSecondaryConnected = false;
    public boolean indexerConnected = false;
    public boolean referenced = false;
    public double pivotDegrees = 0;
    // Both speeds use positive shooting-direction, motor RPM.
    public double shooterPrimaryMotorRpm = 0;
    public double shooterSecondaryMotorRpm = 0;
    public double ageSeconds = 0;
    public double intakeAppliedDuty = 0;
    public double indexerAppliedDuty = 0;
    // Operator and mode inputs are logged with feedback so replay uses the original decisions.
    public double timestampSeconds = 0;
    public boolean enabled = false;
    public boolean commandActive = false;
    public boolean controllerConnected = false;
    public boolean brownedOut = false;
    public boolean intakeRequested = false;
    public boolean shootRequested = false;
  }

  default boolean configure(frc.robot.constants.fuel.FuelConfiguration configuration) {
    return false;
  }

  default boolean confirmUpperReference() {
    return false;
  }

  default void invalidateReference(String reason) {}

  default boolean commissioningReady(FuelCommissioning.Selection selection) {
    return false;
  }

  default void commissioning(FuelCommissioning.Selection selection, double volts) {
    stop();
  }

  default void updateInputs(FuelIOInputs inputs) {}

  /** Implementations must treat pivotEnabled=false as neutral output. */
  default void apply(FuelSequencer.Output output) {}

  default void stop() {
    apply(FuelSequencer.Output.stopped());
  }
}
