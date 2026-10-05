package frc.robot.subsystems.fuel;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.fuel.PreloadAutoCommand;
import frc.robot.commands.fuel.PreloadAutoCommand.Stage;
import frc.robot.constants.fuel.FuelConfiguration;
import frc.robot.constants.fuel.FuelConstants;
import frc.robot.sim.fuel.FuelIOSim;
import frc.robot.subsystems.tank.TankDrive;
import frc.robot.subsystems.tank.TankDriveIO;
import java.util.EnumSet;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

class PreloadAutoTest {
  private static class DriveIO implements TankDriveIO {
    double left, right;

    @Override
    public void setVoltage(double left, double right) {
      this.left = left;
      this.right = right;
    }
  }

  private static class SimIO implements FuelIO {
    final FuelIOSim sim = new FuelIOSim(FuelConstants.SIMULATION, false);
    FuelSequencer.Output output = FuelSequencer.Output.stopped();
    boolean broken, slow, unreferenced;

    @Override
    public void updateInputs(FuelIOInputs inputs) {
      sim.updateInputs(inputs);
      if (broken) inputs.healthy = false;
      if (slow) inputs.shooterSecondaryMotorRpm = 0;
      if (unreferenced) inputs.referenced = false;
    }

    @Override
    public void apply(FuelSequencer.Output output) {
      this.output = output;
      sim.apply(output);
    }
  }

  private DriveIO driveIO;
  private TankDrive drive;
  private SimIO io;
  private FuelSystem fuel;
  private PreloadAutoCommand command;
  private double now;

  @BeforeAll
  static void hal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void setup() {
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    driveIO = new DriveIO();
    drive = new TankDrive(driveIO);
    io = new SimIO();
    fuel = new FuelSystem(io, FuelConfiguration.simulation(FuelConstants.SIMULATION), () -> now);
    fuel.periodic();
    command = new PreloadAutoCommand(drive, fuel, () -> now);
  }

  @AfterEach
  void cleanup() {
    command.end(true);
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterSubsystem(fuel, drive);
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  private void loop() {
    now += .02;
    fuel.periodic();
    command.execute();
  }

  private void until(Stage stage) {
    for (int i = 0; i < 500 && command.stage() != stage && !command.isFinished(); i++) loop();
    assertEquals(stage, command.stage(), command.reason());
  }

  private void stopped() {
    assertEquals(0, driveIO.left);
    assertEquals(0, driveIO.right);
    assertEquals(FuelSequencer.Output.stopped(), io.output);
  }

  @Test
  void fullSimulationBacksStopsSettlesSpinsFeedsThenStops() {
    assertTrue(command.getRequirements().containsAll(java.util.Set.of(drive, fuel)));
    command.initialize();
    var visited = EnumSet.noneOf(Stage.class);
    boolean backed = false, fed = false;
    for (int i = 0; i < 600 && !command.isFinished(); i++) {
      loop();
      visited.add(command.stage());
      backed |= driveIO.left < 0 && driveIO.right < 0;
      fed |= io.output.indexerDuty() > 0;
      if (command.stage() == Stage.SHOOTING) {
        assertEquals(0, driveIO.left);
        assertEquals(0, driveIO.right);
      }
    }
    assertEquals(Stage.DONE, command.stage(), command.reason());
    assertTrue(backed);
    assertTrue(fed);
    assertTrue(
        visited.containsAll(EnumSet.of(Stage.BACKING, Stage.SETTLING, Stage.SHOOTING, Stage.DONE)));
    stopped();
  }

  @ParameterizedTest
  @EnumSource(
      value = Stage.class,
      names = {"BACKING", "SETTLING", "SHOOTING"})
  void faultsAbortEveryActiveStage(Stage stage) {
    command.initialize();
    until(stage);
    io.broken = true;
    loop();
    assertEquals(Stage.ABORTED, command.stage());
    stopped();
  }

  @Test
  void shooterNeverReadyAbortsWithoutFeeding() {
    command.initialize();
    io.slow = true;
    for (int i = 0; i < 600 && !command.isFinished(); i++) {
      loop();
      assertEquals(0, io.output.indexerDuty());
    }
    assertEquals(Stage.ABORTED, command.stage());
    assertTrue(command.reason().contains("Shooter"));
    stopped();
  }

  @Test
  void disableAndCancellationStopBothSubsystems() {
    command.initialize();
    loop();
    assertTrue(driveIO.left < 0);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    loop();
    assertEquals(Stage.ABORTED, command.stage());
    stopped();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    fuel.periodic();
    command.initialize();
    loop();
    command.end(true);
    stopped();
  }

  @Test
  void missingReferencePreventsAnyBacking() {
    io.unreferenced = true;
    fuel.periodic();
    command.initialize();
    assertEquals(Stage.ABORTED, command.stage());
    stopped();
  }

  @Test
  void unverifiedRealAutoCannotMove() {
    CommandScheduler.getInstance().unregisterSubsystem(fuel);
    fuel = new FuelSystem(io, FuelConfiguration.unconfigured(), () -> now);
    fuel.periodic();
    command = new PreloadAutoCommand(drive, fuel, () -> now);
    command.initialize();
    assertEquals(Stage.ABORTED, command.stage());
    stopped();
  }

  @Test
  void recoveryPausesCountTowardFeedDuration() {
    command.initialize();
    until(Stage.SHOOTING);
    for (int i = 0; i < 200 && !fuel.feeding(); i++) loop();
    assertTrue(fuel.feeding());
    loop();
    double feedingAt = now;
    io.slow = true;
    for (int i = 0; i < 200 && !command.isFinished(); i++) loop();
    assertEquals(Stage.DONE, command.stage());
    assertTrue(now - feedingAt < 2.1);
    stopped();
  }

  @Test
  void overallTimeoutStopsEvenBeforeBackingCompletes() {
    command.initialize();
    now = 10;
    fuel.requestAutonomous(false);
    fuel.periodic();
    command.execute();
    assertEquals(Stage.ABORTED, command.stage());
    assertTrue(command.reason().contains("timeout"));
    stopped();
  }
}
