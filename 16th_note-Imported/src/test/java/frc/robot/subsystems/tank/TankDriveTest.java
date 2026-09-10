package frc.robot.subsystems.tank;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.tank.TankDriveIOSim;
import frc.robot.vision.TankVisionDriveAdapter;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mecorobotics.gamepiecevision.GamePieceVisionClient.DriveRequest;

class TankDriveTest {
  private FakeIO io;
  private TankDrive drive;

  private static class FakeIO implements TankDriveIO {
    double left;
    double right;

    public void setVoltage(double left, double right) {
      this.left = left;
      this.right = right;
    }
  }

  @BeforeAll
  static void initHAL() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void setup() {
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    io = new FakeIO();
    drive = new TankDrive(io);
  }

  @AfterEach
  void cleanup() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterSubsystem(drive);
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  @Test
  void tankCanDriveForwardReverseAndTurn() {
    drive.tankDrive(1, 1);
    assertEquals(12, io.left);
    assertEquals(12, io.right);
    drive.tankDrive(-1, -1);
    assertEquals(-12, io.left);
    assertEquals(-12, io.right);
    drive.tankDrive(-1, 1);
    assertEquals(-12, io.left);
    assertEquals(12, io.right);
    drive.tankDrive(0.01, -0.01);
    assertEquals(0, io.left);
    assertEquals(0, io.right);
  }

  @Test
  void ps4ArcadeControlsAndModeTransitions() {
    var controller = new edu.wpi.first.wpilibj.PS4Controller(0);
    var controllerSim = new edu.wpi.first.wpilibj.simulation.PS4ControllerSim(controller);
    try (var vision = new org.mecorobotics.gamepiecevision.GamePieceVisionClient("teleop-test")) {
      var command = new frc.robot.commands.drive.TankTeleopCommand(drive, controller, vision);
      assertTrue(command.getRequirements().contains(drive));
      controllerSim.setLeftY(-1);
      controllerSim.notifyNewData();
      command.execute();
      assertEquals(12, io.left);
      assertEquals(12, io.right);

      controllerSim.setLeftY(0);
      controllerSim.setRightX(1);
      controllerSim.notifyNewData();
      command.execute();
      assertEquals(12, io.left);
      assertEquals(-12, io.right);

      controllerSim.setRightX(0.01);
      controllerSim.notifyNewData();
      command.execute();
      assertEquals(0, io.left);
      assertEquals(0, io.right);

      controllerSim.setLeftY(-1);
      controllerSim.notifyNewData();
      command.execute();
      command.end(true);
      assertEquals(0, io.left);
      assertEquals(0, io.right);

      DriverStationSim.setAutonomous(true);
      DriverStationSim.notifyNewData();
      command.execute();
      assertEquals(0, io.left);
      assertEquals(0, io.right);
    }
  }

  @Test
  void manualArcadeMixesAndSquaresInputs() {
    drive.arcadeDriveManual(1, 1);
    assertEquals(0, io.left);
    assertEquals(12, io.right);
    drive.arcadeDriveManual(-1, 0);
    assertEquals(-12, io.left);
    assertEquals(-12, io.right);
    drive.arcadeDriveManual(0.51, 0);
    assertEquals(3, io.left, 1e-9);
    assertEquals(3, io.right, 1e-9);
  }

  @Test
  void outputsAreBoundedAndNonfiniteVoltageStopsBothSides() {
    drive.setVoltage(99, -99);
    assertEquals(12, io.left);
    assertEquals(-12, io.right);
    drive.setVoltage(Double.NaN, 12);
    assertEquals(0, io.left);
    assertEquals(0, io.right);
  }

  @Test
  void disableStopsExistingOutputAndRejectsNewOutput() {
    drive.tankDrive(1, 1);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    drive.periodic();
    assertEquals(0, io.left);
    assertEquals(0, io.right);
    drive.tankDrive(1, 1);
    assertEquals(0, io.left);
    assertEquals(0, io.right);
  }

  @Test
  void interruptedDriveCommandStopsOutputs() {
    var command = drive.runEnd(() -> drive.tankDrive(1, 1), drive::stop);
    command.schedule();
    CommandScheduler.getInstance().run();
    assertEquals(12, io.left);
    command.cancel();
    assertEquals(0, io.left);
    assertEquals(0, io.right);
  }

  @Test
  void visionIsLimitedAndGoalOrInactiveRequestsStop() {
    var adapter = new TankVisionDriveAdapter(drive, 0.35, 0.35);
    adapter.apply(new DriveRequest(true, false, 1, 0, 0, 1));
    assertEquals(4.2, io.left, 1e-9);
    assertEquals(4.2, io.right, 1e-9);
    adapter.apply(new DriveRequest(true, true, 1, 0, 0, 2));
    assertEquals(0, io.left);
    assertEquals(0, io.right);
    adapter.apply(DriveRequest.inactive());
    assertEquals(0, io.left);
  }

  @Test
  void positiveVisionTurnIsCounterclockwise() {
    var adapter = new TankVisionDriveAdapter(drive, 0.35, 0.35);
    adapter.apply(new DriveRequest(true, false, 0, 0, 0.5, 1));
    assertEquals(-2.1, io.left, 1e-9);
    assertEquals(2.1, io.right, 1e-9);
  }

  @Test
  void simulationMovesForwardAndTurns() {
    var sim = new TankDriveIOSim();
    var inputs = new TankDriveIO.TankDriveIOInputs();
    sim.setVoltage(6, 6);
    for (int i = 0; i < 50; i++) sim.updateInputs(inputs);
    assertTrue(inputs.hasPositionFeedback);
    assertTrue(inputs.leftPositionMeters > 0.1);
    assertEquals(inputs.leftPositionMeters, inputs.rightPositionMeters, 1e-6);
    sim.setVoltage(-6, 6);
    for (int i = 0; i < 50; i++) sim.updateInputs(inputs);
    assertTrue(Math.abs(inputs.headingRadians) > 0.1);
  }
}
