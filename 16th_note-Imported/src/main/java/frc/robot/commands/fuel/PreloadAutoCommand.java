package frc.robot.commands.fuel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.fuel.FuelConfiguration.Auto;
import frc.robot.subsystems.fuel.FuelSystem;
import frc.robot.subsystems.tank.TankDrive;
import java.util.function.DoubleSupplier;

/** Timed backing is intentionally not represented as a distance or heading-controlled action. */
public final class PreloadAutoCommand extends Command {
  public enum Stage {
    BACKING,
    SETTLING,
    SHOOTING,
    DONE,
    ABORTED
  }

  private final TankDrive drive;
  private final FuelSystem fuel;
  private final DoubleSupplier clock;
  private Auto config;
  private Stage stage = Stage.ABORTED;
  private double start, stageStart, lastUpdate, feedSeconds;
  private String reason = "Not started";

  public PreloadAutoCommand(TankDrive drive, FuelSystem fuel) {
    this(drive, fuel, Timer::getFPGATimestamp);
  }

  public PreloadAutoCommand(TankDrive drive, FuelSystem fuel, DoubleSupplier clock) {
    this.drive = drive;
    this.fuel = fuel;
    this.clock = clock;
    addRequirements(drive, fuel);
    setName("Back up and shoot preloads");
  }

  @Override
  public void initialize() {
    drive.stop();
    fuel.stop();
    start = clock.getAsDouble();
    stageStart = start;
    lastUpdate = start;
    feedSeconds = 0;
    config = fuel.autonomousConfiguration();
    String blocked = fuel.autonomousBlockReason();
    if (!DriverStation.isAutonomousEnabled() || !Double.isFinite(start)) {
      abort("Not enabled in autonomous or invalid clock");
      return;
    }
    if (!blocked.isEmpty()) {
      abort(blocked);
      return;
    }
    stage = Stage.BACKING;
    reason = "";
    fuel.armAutonomous();
    fuel.requestAutonomous(false);
    publish();
  }

  @Override
  public void execute() {
    if (isFinished()) return;
    double now = clock.getAsDouble();
    if (!DriverStation.isAutonomousEnabled()) {
      abort("Autonomous disabled/interrupted");
      return;
    }
    if (!Double.isFinite(now) || now < lastUpdate || now - start >= config.totalSeconds()) {
      abort("Overall autonomous timeout");
      return;
    }
    String blocked = fuel.automaticBlockReason();
    if (!blocked.isEmpty()) {
      abort(blocked);
      return;
    }
    double elapsed = now - lastUpdate;
    lastUpdate = now;
    switch (stage) {
      case BACKING -> {
        fuel.requestAutonomous(false);
        if (now - stageStart >= config.backSeconds()) {
          drive.stop();
          stage = Stage.SETTLING;
          stageStart = now;
        } else drive.setVoltage(config.leftVolts(), config.rightVolts());
      }
      case SETTLING -> {
        drive.stop();
        fuel.requestAutonomous(false);
        if (now - stageStart >= config.settleSeconds()) {
          stage = Stage.SHOOTING;
          stageStart = now;
        }
      }
      case SHOOTING -> {
        drive.stop();
        fuel.requestAutonomous(true);
        if (fuel.feeding()) feedSeconds += elapsed;
        if (feedSeconds >= config.feedSeconds()) {
          stage = Stage.DONE;
          drive.stop();
          fuel.stop();
        }
      }
      default -> {}
    }
    publish();
  }

  private void abort(String why) {
    stage = Stage.ABORTED;
    reason = why;
    drive.stop();
    fuel.stop();
    publish();
  }

  private void publish() {
    SmartDashboard.putString("Fuel/Auto/Stage", stage.name());
    SmartDashboard.putString("Fuel/Auto/Reason", reason);
    SmartDashboard.putNumber("Fuel/Auto/FeedSeconds", feedSeconds);
  }

  @Override
  public boolean isFinished() {
    return stage == Stage.DONE || stage == Stage.ABORTED;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) abort("Command interrupted");
    else {
      drive.stop();
      fuel.stop();
    }
  }

  public Stage stage() {
    return stage;
  }

  public String reason() {
    return reason;
  }
}
