package frc.robot.subsystems.fuel;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.constants.fuel.FuelConfiguration;
import frc.robot.constants.fuel.FuelConstants;
import frc.robot.sim.fuel.*;
import org.junit.jupiter.api.Test;

class FuelShotPracticeTest {
  @Test
  void bothTrenchPresetsScoreSixBelowNineteenFeetAndExceedCloseLimit() {
    for (var preset :
        new FuelShotPractice.Preset[] {FuelShotPractice.LEFT, FuelShotPractice.RIGHT}) {
      double peak =
          FuelWorldSim.predictedPeakMeters(
              preset.motorRpm(), FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS);
      assertTrue(peak > 9 * .3048);
      assertTrue(peak < 19 * .3048);
      var world = new FuelWorldSim();
      world.reset(6);
      world.update(
          .02,
          preset.pose(),
          true,
          false,
          true,
          preset.motorRpm(),
          FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS);
      assertEquals(6, world.held());
      for (int i = 0; i < 400; i++) {
        world.update(
            .02,
            preset.pose(),
            true,
            false,
            true,
            preset.motorRpm(),
            FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS,
            preset.peakLimitFeet() * .3048);
        for (var shot : world.projectilePoses())
          assertTrue(shot.getZ() + FuelWorldSim.RADIUS <= 19 * .3048);
      }
      assertEquals(6, world.scored());
      assertEquals(0, world.held());
    }
  }

  @Test
  void presetTargetsAreDisabledOnlyAndCannotConfigureRealIo() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    var config = FuelConfiguration.simulation(FuelConstants.SIMULATION);
    var sim = new FuelSystem(new FuelIOSim(FuelConstants.SIMULATION, false), config, () -> 0);
    var real = new FuelSystem(new FuelIO() {}, config, () -> 0);
    try {
      assertFalse(real.setSimulationShooterTarget(FuelShotPractice.LEFT.motorRpm()));
      assertTrue(sim.setSimulationShooterTarget(FuelShotPractice.LEFT.motorRpm()));
      assertTrue(sim.setSimulationShooterTarget(FuelShotPractice.CLOSE.motorRpm()));
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      assertFalse(sim.setSimulationShooterTarget(FuelShotPractice.RIGHT.motorRpm()));
    } finally {
      sim.stop();
      real.stop();
      edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().unregisterSubsystem(sim, real);
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    }
  }

  @Test
  void dashboardPresetsRequireOptInAndDisabledAndCloseRestoresNineFeet() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    var fuel =
        new FuelSystem(
            new FuelIOSim(FuelConstants.SIMULATION, false),
            FuelConfiguration.simulation(FuelConstants.SIMULATION),
            () -> 0);
    var drive = new frc.robot.subsystems.tank.TankDrive(new frc.robot.sim.tank.TankDriveIOSim());
    var practice = new FuelPracticeSimulation(drive, fuel);
    var trench =
        (edu.wpi.first.wpilibj2.command.Command)
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getData(
                "Fuel/Sim/Prepare left trench shot (disabled)");
    var close =
        (edu.wpi.first.wpilibj2.command.Command)
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getData(
                "Fuel/Sim/Reset six preloads (disabled)");
    try {
      trench.initialize();
      assertTrue(
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString("Fuel/Sim/ResetStatus", "")
              .contains("Rejected"));
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean(
          "Fuel/Sim/TrenchShotsEnabled", true);
      trench.initialize();
      assertEquals(
          19,
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber(
              "Fuel/Sim/PeakLimitFeet", 0));
      assertEquals(FuelShotPractice.LEFT.pose(), drive.simulatedPose());
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      close.initialize();
      assertEquals(
          19,
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber(
              "Fuel/Sim/PeakLimitFeet", 0));
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      close.initialize();
      assertEquals(
          9,
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber(
              "Fuel/Sim/PeakLimitFeet", 0));
      assertFalse(
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean(
              "Fuel/Sim/TrenchShotsEnabled", true));
      assertEquals(
          3000,
          edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber(
              "Fuel/Config/PrimaryMotorRpm", 0));
    } finally {
      fuel.stop();
      drive.stop();
      edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance()
          .unregisterSubsystem(fuel, drive);
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    }
  }
}
