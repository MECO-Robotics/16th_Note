package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.vision.HubVision;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Runs the flywheel from a measured three-point distance calibration while Y is held. */
public final class DistanceBasedShooterCommand extends Command {
  private final Flywheel shooter;
  private final HubVision vision;
  private final DoubleSupplier closeDistanceMeters;
  private final DoubleSupplier closeRps;
  private final DoubleSupplier middleDistanceMeters;
  private final DoubleSupplier middleRps;
  private final DoubleSupplier farDistanceMeters;
  private final DoubleSupplier farRps;
  private final DoubleSupplier maxObservationAgeSeconds;
  private final DoubleSupplier maxPoseAmbiguity;
  private final DoubleSupplier lostTargetHoldSeconds;
  private final double maximumRps;
  private double lastValidRps;
  private double lastValidTimestampSeconds;

  public DistanceBasedShooterCommand(
      Flywheel shooter,
      HubVision vision,
      DoubleSupplier closeDistanceMeters,
      DoubleSupplier closeRps,
      DoubleSupplier middleDistanceMeters,
      DoubleSupplier middleRps,
      DoubleSupplier farDistanceMeters,
      DoubleSupplier farRps,
      DoubleSupplier maxObservationAgeSeconds,
      DoubleSupplier maxPoseAmbiguity,
      DoubleSupplier lostTargetHoldSeconds,
      double maximumRps) {
    this.shooter = shooter;
    this.vision = vision;
    this.closeDistanceMeters = closeDistanceMeters;
    this.closeRps = closeRps;
    this.middleDistanceMeters = middleDistanceMeters;
    this.middleRps = middleRps;
    this.farDistanceMeters = farDistanceMeters;
    this.farRps = farRps;
    this.maxObservationAgeSeconds = maxObservationAgeSeconds;
    this.maxPoseAmbiguity = maxPoseAmbiguity;
    this.lostTargetHoldSeconds = lostTargetHoldSeconds;
    this.maximumRps = maximumRps;
    addRequirements(shooter);
    setName("Distance-based shooter");
  }

  @Override
  public void initialize() {
    lastValidRps = 0.0;
    lastValidTimestampSeconds = Double.NEGATIVE_INFINITY;
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    HubVision.Observation observation = vision.getObservation();
    DistanceBasedShooterController.Output output =
        DistanceBasedShooterController.calculate(
            observation.distanceMeters(),
            closeDistanceMeters.getAsDouble(),
            closeRps.getAsDouble(),
            middleDistanceMeters.getAsDouble(),
            middleRps.getAsDouble(),
            farDistanceMeters.getAsDouble(),
            farRps.getAsDouble(),
            maximumRps);
    boolean usableObservation =
        vision.isUsableForDistance(
            observation,
            nonnegative(maxObservationAgeSeconds.getAsDouble()),
            nonnegative(maxPoseAmbiguity.getAsDouble()));

    String status;
    double commandedRps = 0.0;
    if (!output.configured()) {
      lastValidTimestampSeconds = Double.NEGATIVE_INFINITY;
      shooter.stop();
      status = "UNCONFIGURED";
    } else if (usableObservation && output.inRange()) {
      commandedRps = output.targetRps();
      lastValidRps = commandedRps;
      lastValidTimestampSeconds = now;
      shooter.setVelocity(commandedRps);
      status = "TRACKING";
    } else if (usableObservation) {
      lastValidTimestampSeconds = Double.NEGATIVE_INFINITY;
      shooter.stop();
      status = "OUT_OF_RANGE";
    } else if (lastValidRps > 0.0
        && now - lastValidTimestampSeconds <= nonnegative(lostTargetHoldSeconds.getAsDouble())) {
      commandedRps = lastValidRps;
      shooter.setVelocity(commandedRps);
      status = "HOLDING_LAST";
    } else {
      shooter.stop();
      status = "NO_VALID_TARGET";
    }

    Logger.recordOutput("DistanceShooter/Active", true);
    Logger.recordOutput("DistanceShooter/Status", status);
    Logger.recordOutput("DistanceShooter/MeasuredDistanceMeters", observation.distanceMeters());
    Logger.recordOutput("DistanceShooter/TargetRps", commandedRps);
    Logger.recordOutput("DistanceShooter/AtSpeed", commandedRps > 0.0 && shooter.isFinished());
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    Logger.recordOutput("DistanceShooter/Active", false);
    Logger.recordOutput("DistanceShooter/Status", "INACTIVE");
    Logger.recordOutput("DistanceShooter/TargetRps", 0.0);
    Logger.recordOutput("DistanceShooter/AtSpeed", false);
  }

  private static double nonnegative(double value) {
    return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
  }
}
