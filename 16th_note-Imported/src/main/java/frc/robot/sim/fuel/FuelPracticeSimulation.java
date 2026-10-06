package frc.robot.sim.fuel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.fuel.FuelSystem;
import frc.robot.subsystems.tank.TankDrive;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

/** Constructed only in SIM; visual estimates cannot configure or enable real hardware. */
public final class FuelPracticeSimulation {
  private boolean trenchShotActive;
  private final FuelWorldSim world = new FuelWorldSim();
  private final TankDrive drive;
  private final FuelSystem fuel;
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1, 1);
  private final LoggedMechanismLigament2d intake =
      mechanism.getRoot("Pivot", .5, .3).append(new LoggedMechanismLigament2d("Intake", .4, 90));

  public FuelPracticeSimulation(TankDrive drive, FuelSystem fuel) {
    this.drive = drive;
    this.fuel = fuel;
    SmartDashboard.putBoolean("Fuel/Sim/TrenchShotsEnabled", false);
    SmartDashboard.putString("Fuel/Sim/ShotPreset", "Close");
    SmartDashboard.putData(
        "Fuel/Sim/Prepare left trench shot (disabled)",
        Commands.runOnce(() -> prepareShot(FuelShotPractice.LEFT), drive, fuel)
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Fuel/Sim/Prepare right trench shot (disabled)",
        Commands.runOnce(() -> prepareShot(FuelShotPractice.RIGHT), drive, fuel)
            .ignoringDisable(true));
    SmartDashboard.putNumber("Fuel/Sim/LaunchSpeedMps", FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS);
    SmartDashboard.putNumber(
        "Fuel/Sim/PeakLimitFeet", FuelWorldSim.DEFAULT_PEAK_LIMIT_METERS / .3048);
    SmartDashboard.putString("Fuel/Sim/Hood", "Fixed at 80 degrees above horizontal");
    SmartDashboard.putString(
        "Fuel/Sim/Model",
        "Practice estimates; basic chassis/wall/hub collisions; no active-hub rules");
    SmartDashboard.putData(
        "Fuel/Sim/Reset empty (disabled)",
        Commands.runOnce(() -> reset(0, FuelWorldSim.START, true), drive, fuel)
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Fuel/Sim/Reset six preloads (disabled)",
        Commands.runOnce(() -> prepareShot(FuelShotPractice.CLOSE), drive, fuel)
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Fuel/Sim/Prepare pickup practice (disabled)",
        Commands.runOnce(() -> reset(0, FuelWorldSim.PICKUP_START, true), drive, fuel)
            .ignoringDisable(true));
    reset(0, FuelWorldSim.START, false);
  }

  private void prepareShot(FuelShotPractice.Preset preset) {
    if (!DriverStation.isDisabled()) {
      SmartDashboard.putString("Fuel/Sim/ResetStatus", "Rejected: disable simulation first");
      return;
    }
    boolean trench = preset != FuelShotPractice.CLOSE;
    if (trench && !SmartDashboard.getBoolean("Fuel/Sim/TrenchShotsEnabled", false)) {
      SmartDashboard.putString(
          "Fuel/Sim/ResetStatus", "Rejected: enable TrenchShotsEnabled first (SIM only)");
      return;
    }
    if (!fuel.setSimulationShooterTarget(preset.motorRpm())) {
      SmartDashboard.putString("Fuel/Sim/ResetStatus", "Rejected: could not apply SIM shot preset");
      return;
    }
    trenchShotActive = trench;
    if (!trench) SmartDashboard.putBoolean("Fuel/Sim/TrenchShotsEnabled", false);
    SmartDashboard.putNumber("Fuel/Sim/LaunchSpeedMps", FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS);
    SmartDashboard.putNumber("Fuel/Sim/PeakLimitFeet", preset.peakLimitFeet());
    SmartDashboard.putString("Fuel/Sim/ShotPreset", preset.name() + " (SIM only; uncalibrated)");
    reset(6, preset.pose(), true);
  }

  private void reset(int preloads, edu.wpi.first.math.geometry.Pose2d pose, boolean reference) {
    if (!DriverStation.isDisabled()) {
      SmartDashboard.putString("Fuel/Sim/ResetStatus", "Rejected: disable simulation first");
      return;
    }
    drive.stop();
    fuel.stop();
    drive.resetSimulationPose(pose);
    world.reset(preloads);
    boolean ready = reference && fuel.confirmUpperReference();
    SmartDashboard.putString(
        "Fuel/Sim/ResetStatus",
        ready
            ? "400 neutral-zone fuel reset; simulated intake referenced; enable Teleop and release bumpers"
            : "400 neutral-zone fuel reset; reference intake before enabling");
  }

  public void periodic() {
    var pose = drive.simulatedPose();
    boolean trenchAllowed =
        !trenchShotActive || SmartDashboard.getBoolean("Fuel/Sim/TrenchShotsEnabled", false);
    world.update(
        .02,
        pose,
        DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled(),
        fuel.collecting(),
        fuel.feeding() && trenchAllowed,
        fuel.simulatedShooterRpm(),
        SmartDashboard.getNumber("Fuel/Sim/LaunchSpeedMps", FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS),
        SmartDashboard.getNumber("Fuel/Sim/PeakLimitFeet", Double.NaN) * .3048);
    SmartDashboard.putString(
        "Fuel/Sim/ShotBlockedBy",
        trenchAllowed
            ? world.shotBlockedReason()
            : "Trench shots disabled; select close reset to restore close settings");
    SmartDashboard.putNumber(
        "Fuel/Sim/PredictedPeakFeet",
        FuelWorldSim.predictedPeakMeters(
                fuel.simulatedShooterRpm(),
                SmartDashboard.getNumber("Fuel/Sim/LaunchSpeedMps", Double.NaN))
            / .3048);
    Logger.recordOutput("Visualization/16thNote/RobotPose", pose);
    var mouth = FuelWorldSim.pickupMouth(pose);
    Logger.recordOutput(
        "Simulation/Fuel/PickupMouth",
        new edu.wpi.first.math.geometry.Pose3d(
            mouth.getX(),
            mouth.getY(),
            FuelWorldSim.RADIUS,
            new edu.wpi.first.math.geometry.Rotation3d()));
    String blocked = fuel.automaticBlockReason();
    String pickupStatus =
        !DriverStation.isTeleopEnabled()
            ? "Enable Teleop for pickup"
            : !blocked.isEmpty()
                ? blocked
                : world.held() >= 6
                    ? "Full: 6/6 fuel"
                    : !fuel.collecting()
                        ? "Hold LB/L1; release both bumpers first after enable; wait for intake to lower"
                        : world.nearestFuelDistance(pose) > FuelWorldSim.PICKUP_RADIUS
                                || world.nearestFuelDistance(pose) < 0
                            ? "Intake running: drive the pickup marker onto fuel"
                            : "Collecting";
    SmartDashboard.putString("Fuel/Sim/PickupStatus", pickupStatus);
    SmartDashboard.putBoolean("Fuel/Sim/PickupActive", fuel.collecting());
    SmartDashboard.putNumber("Fuel/Sim/NearestFuelMeters", world.nearestFuelDistance(pose));
    SmartDashboard.putNumber("Fuel/Sim/GroundCount", world.groundCount());
    Logger.recordOutput("Simulation/Fuel/PickupStatus", pickupStatus);
    Logger.recordOutput("Simulation/Fuel/GroundCount", world.groundCount());
    intake.setAngle(90 - fuel.pivotDegrees());
    Logger.recordOutput("Visualization/16thNote/Intake", mechanism);
    Logger.recordOutput(
        "Visualization/16thNote/ComponentPoses",
        FuelVisualization.componentPoses(fuel.pivotDegrees()));
    Logger.recordOutput("Simulation/Fuel/GroundPoses", world.groundPoses());
    Logger.recordOutput("Simulation/Fuel/ProjectilePoses", world.projectilePoses());
    Logger.recordOutput("Simulation/Fuel/HeldPoses", world.heldPoses(pose));
    Logger.recordOutput("Simulation/Fuel/HeldCount", world.held());
    Logger.recordOutput("Simulation/Fuel/ScoredCount", world.scored());
    Logger.recordOutput("Simulation/Fuel/LaunchedCount", world.launched());
    SmartDashboard.putNumber("Fuel/Sim/HeldCount", world.held());
    SmartDashboard.putNumber("Fuel/Sim/ScoredCount", world.scored());
  }
}
