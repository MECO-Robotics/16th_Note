package frc.robot.subsystems.vision;

/** Camera adapter used by the distance-based shooter. PhotonVision supplies this implementation. */
@FunctionalInterface
public interface HubVisionIO {
  /** Latest hub-tag observation expressed from Pete's center and robot coordinate frame. */
  final class HubVisionIOInputs {
    public boolean connected;
    public boolean hasTarget;
    public int fiducialId = -1;
    public double distanceMeters = Double.NaN;
    /** Positive when the robot must turn counterclockwise to center the tag. */
    public double yawDegrees = Double.NaN;

    public double poseAmbiguity = Double.NaN;
    /** PhotonVision capture timestamp, expressed in FPGA seconds. */
    public double captureTimestampSeconds = Double.NaN;
  }

  void updateInputs(HubVisionIOInputs inputs);

  static HubVisionIO disconnected() {
    return inputs -> {};
  }
}
