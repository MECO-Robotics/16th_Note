package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.motor.Motor;
import frc.robot.subsystems.position_joint.PositionJoint;
import java.util.function.DoubleSupplier;

/** Small, tunable autonomous actions that do not require drivetrain odometry. */
public final class PreloadAutoCommands {
  private PreloadAutoCommands() {}

  public static Command moveIntake(
      PositionJoint pivot, DoubleSupplier position, DoubleSupplier timeoutSeconds, String name) {
    return Commands.defer(
            () ->
                Commands.run(() -> pivot.setPosition(position.getAsDouble()), pivot)
                    .until(pivot::isFinished)
                    .withTimeout(MathUtil.clamp(timeoutSeconds.getAsDouble(), 0.0, 4.0)),
            java.util.Set.of(pivot))
        .withName(name);
  }

  public static Command shootPreloads(
      Flywheel shooter,
      PositionJoint pivot,
      Motor indexer,
      DoubleSupplier shooterRps,
      DoubleSupplier feedPosition,
      DoubleSupplier loweredPosition,
      DoubleSupplier spinupSeconds,
      DoubleSupplier feedSeconds,
      DoubleSupplier indexerVolts,
      DoubleSupplier pivotTimeoutSeconds) {
    return Commands.defer(
            () -> {
              double speed = Math.max(0.0, shooterRps.getAsDouble());
              double spinup = MathUtil.clamp(spinupSeconds.getAsDouble(), 0.0, 6.0);
              double feed = MathUtil.clamp(feedSeconds.getAsDouble(), 0.0, 6.0);
              double feedVolts = MathUtil.clamp(indexerVolts.getAsDouble(), -6.0, 6.0);
              Command holdShooter = Commands.run(() -> shooter.setVelocity(speed), shooter);
              Command holdFeedPosition =
                  Commands.run(() -> pivot.setPosition(feedPosition.getAsDouble()), pivot);
              return Commands.sequence(
                      Commands.deadline(
                          Commands.waitSeconds(spinup), holdShooter, holdFeedPosition),
                      Commands.deadline(
                          Commands.waitSeconds(feed),
                          Commands.run(() -> shooter.setVelocity(speed), shooter),
                          Commands.run(() -> pivot.setPosition(feedPosition.getAsDouble()), pivot),
                          indexer.voltageCommand(() -> feedVolts)),
                      Commands.runOnce(shooter::stop, shooter),
                      moveIntake(
                          pivot,
                          loweredPosition,
                          pivotTimeoutSeconds,
                          "Lower Intake After Shooting"))
                  .finallyDo(
                      interrupted -> {
                        shooter.stop();
                        indexer.stop();
                        if (interrupted) pivot.stop();
                      });
            },
            java.util.Set.of(shooter, pivot, indexer))
        .withName("Shoot");
  }

  public static Command shootFullSpeedWhenReady(
      Flywheel shooter,
      PositionJoint pivot,
      Motor indexer,
      DoubleSupplier shooterRps,
      DoubleSupplier agitationPosition,
      DoubleSupplier readinessTimeoutSeconds,
      DoubleSupplier feedSeconds,
      DoubleSupplier indexerVolts) {
    return Commands.defer(
            () -> {
              double speed = Math.max(0.0, shooterRps.getAsDouble());
              double readinessTimeout =
                  MathUtil.clamp(readinessTimeoutSeconds.getAsDouble(), 0.0, 8.0);
              double feed = MathUtil.clamp(feedSeconds.getAsDouble(), 0.0, 6.0);
              double feedVolts = MathUtil.clamp(indexerVolts.getAsDouble(), -6.0, 6.0);
              Command waitForSpeed =
                  Commands.run(
                          () -> {
                            shooter.setVelocity(speed);
                            pivot.setPosition(agitationPosition.getAsDouble());
                          },
                          shooter,
                          pivot)
                      .until(shooter::isFinished)
                      .withTimeout(readinessTimeout);
              Command feedWhenReady =
                  Commands.either(
                      Commands.deadline(
                          Commands.waitSeconds(feed),
                          Commands.run(() -> shooter.setVelocity(speed), shooter),
                          Commands.run(
                              () -> pivot.setPosition(agitationPosition.getAsDouble()), pivot),
                          indexer.voltageCommand(() -> feedVolts)),
                      Commands.runOnce(
                          () ->
                              DriverStation.reportError(
                                  "ShootFullSpeed timed out before the flywheel reached speed; indexer was not started.",
                                  false)),
                      shooter::isFinished);
              return Commands.sequence(waitForSpeed, feedWhenReady)
                  .finallyDo(
                      interrupted -> {
                        shooter.stop();
                        indexer.stop();
                        if (interrupted) pivot.stop();
                      });
            },
            java.util.Set.of(shooter, pivot, indexer))
        .withName("ShootFullSpeed");
  }
}
