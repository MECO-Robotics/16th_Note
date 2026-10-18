package frc.robot.subsystems.tank;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.sim.tank.TankDriveIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

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
  void xboxArcadeControlsAndModeTransitions() {
    var controllerSim = new edu.wpi.first.wpilibj.simulation.GenericHIDSim(0);
    controllerSim.setAxisCount(6);
    controllerSim.setButtonCount(14);
    controllerSim.setName("Xbox Controller");
    DriverStationSim.setJoystickIsXbox(0, true);
    {
      var command =
          new frc.robot.commands.drive.TankTeleopCommand(drive, new CommandXboxController(0));
      assertTrue(command.getRequirements().contains(drive));
      controllerSim.setRawAxis(1, -1);
      controllerSim.notifyNewData();
      command.execute();
      assertEquals(12, io.left);
      assertEquals(12, io.right);

      controllerSim.setRawAxis(1, 1);
      controllerSim.notifyNewData();
      command.execute();
      assertEquals(-12, io.left);
      assertEquals(-12, io.right);
      controllerSim.setRawAxis(1, 0);
      controllerSim.setRawAxis(4, 1);
      controllerSim.notifyNewData();
      command.execute();
      assertEquals(12, io.left);
      assertEquals(-12, io.right);

      controllerSim.setRawAxis(4, -1);
      controllerSim.notifyNewData();
      command.execute();
      assertEquals(-12, io.left);
      assertEquals(12, io.right);
      controllerSim.setRawAxis(4, 0.01);
      controllerSim.notifyNewData();
      command.execute();
      assertEquals(0, io.left);
      assertEquals(0, io.right);

      controllerSim.setRawAxis(1, -1);
      controllerSim.notifyNewData();
      command.execute();
      controllerSim.setAxisCount(0);
      controllerSim.setButtonCount(0);
      controllerSim.setPOVCount(0);
      controllerSim.notifyNewData();
      command.execute();
      assertEquals(0, io.left);
      assertEquals(0, io.right);
      command.end(true);
      assertEquals(0, io.left);
      assertEquals(0, io.right);
      controllerSim.setAxisCount(6);
      controllerSim.setButtonCount(14);

      DriverStationSim.setAutonomous(true);
      DriverStationSim.notifyNewData();
      command.execute();
      assertEquals(0, io.left);
      assertEquals(0, io.right);
    }
  }

  @Test
  void xboxActionMappings() {
    var sim = new edu.wpi.first.wpilibj.simulation.GenericHIDSim(0);
    sim.setAxisCount(6);
    sim.setButtonCount(14);
    sim.notifyNewData();
    sim.setRawButton(5, true);
    sim.notifyNewData();
    var controls = new CommandXboxController(0);
    assertTrue(controls.leftBumper().getAsBoolean());
    assertFalse(controls.rightBumper().getAsBoolean());
    sim.setRawButton(5, false);
    sim.setRawButton(6, true);
    sim.notifyNewData();
    assertFalse(controls.leftBumper().getAsBoolean());
    assertTrue(controls.rightBumper().getAsBoolean());
    sim.setRawButton(6, false);
    sim.notifyNewData();
    assertFalse(controls.rightBumper().getAsBoolean());
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
    CommandScheduler.getInstance().schedule(command);
    CommandScheduler.getInstance().run();
    assertEquals(12, io.left);
    command.cancel();
    assertEquals(0, io.left);
    assertEquals(0, io.right);
  }

  @Test
  void simulationPoseResetClearsVelocityAndPreservesHeading() {
    var sim = new TankDriveIOSim();
    var inputs = new TankDriveIO.TankDriveIOInputs();
    sim.setVoltage(6, 6);
    for (int i = 0; i < 50; i++) sim.updateInputs(inputs);
    var pose =
        new edu.wpi.first.math.geometry.Pose2d(
            3, 4, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90));
    sim.resetSimulationPose(pose);
    sim.updateInputs(inputs);
    assertEquals(3, inputs.simulatedPose.getX(), 1e-6);
    assertEquals(4, inputs.simulatedPose.getY(), 1e-6);
    assertEquals(Math.PI / 2, inputs.headingRadians, 1e-6);
    assertEquals(0, inputs.leftVelocityMetersPerSecond, 1e-6);
    sim.setVoltage(6, 6);
    for (int i = 0; i < 50; i++) sim.updateInputs(inputs);
    assertTrue(inputs.simulatedPose.getY() > 4.1);
  }

  @Test
  void simulationStopsAtHubAndCanBackAway() {
    var sim = new TankDriveIOSim();
    var inputs = new TankDriveIO.TankDriveIOInputs();
    sim.setVoltage(12, 12);
    for (int i = 0; i < 200; i++) sim.updateInputs(inputs);
    double stoppedX = inputs.simulatedPose.getX();
    assertTrue(stoppedX <= frc.robot.sim.tank.PracticeField.HUB_X - .92);
    assertTrue(stoppedX > frc.robot.sim.tank.PracticeField.START.getX());
    sim.setVoltage(-6, -6);
    for (int i = 0; i < 30; i++) sim.updateInputs(inputs);
    assertTrue(inputs.simulatedPose.getX() < stoppedX - .1);
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
