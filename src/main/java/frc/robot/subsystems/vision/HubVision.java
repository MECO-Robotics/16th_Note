package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Validates and logs the hub-tag observation supplied by the PhotonVision adapter. */
public final class HubVision extends SubsystemBase {
  public record Observation(
      boolean connected,
      boolean hasTarget,
      int fiducialId,
      double distanceMeters,
      double yawDegrees,
      double poseAmbiguity,
      double captureTimestampSeconds) {}

  private final HubVisionIO io;
  private final HubVisionIO.HubVisionIOInputs inputs = new HubVisionIO.HubVisionIOInputs();

  public HubVision(HubVisionIO io) {
    this.io = io;
    setName("HubVision");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Observation observation = getObservation();
    Logger.recordOutput("HubVision/Connected", observation.connected());
    Logger.recordOutput("HubVision/HasTarget", observation.hasTarget());
    Logger.recordOutput("HubVision/FiducialId", observation.fiducialId());
    Logger.recordOutput("HubVision/DistanceMeters", observation.distanceMeters());
    Logger.recordOutput("HubVision/YawDegrees", observation.yawDegrees());
    Logger.recordOutput("HubVision/PoseAmbiguity", observation.poseAmbiguity());
    Logger.recordOutput("HubVision/CaptureTimestampSeconds", observation.captureTimestampSeconds());
  }

  public Observation getObservation() {
    return new Observation(
        inputs.connected,
        inputs.hasTarget,
        inputs.fiducialId,
        inputs.distanceMeters,
        inputs.yawDegrees,
        inputs.poseAmbiguity,
        inputs.captureTimestampSeconds);
  }

  public boolean isUsableForDistance(
      Observation observation, double maxAgeSeconds, double maxAmbiguity) {
    double ageSeconds = Timer.getFPGATimestamp() - observation.captureTimestampSeconds();
    return observation.connected()
        && observation.hasTarget()
        && isHubTag(observation.fiducialId())
        && Double.isFinite(observation.distanceMeters())
        && observation.distanceMeters() > 0.0
        && Double.isFinite(observation.poseAmbiguity())
        && observation.poseAmbiguity() >= 0.0
        && observation.poseAmbiguity() <= maxAmbiguity
        && Double.isFinite(ageSeconds)
        && ageSeconds >= 0.0
        && ageSeconds <= maxAgeSeconds;
  }

  /** 2026 REBUILT tags mounted to the blue and red hubs. */
  public static boolean isHubTag(int fiducialId) {
    return switch (fiducialId) {
      case 2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27 -> true;
      default -> false;
    };
  }
}
