package frc.robot.subsystems.tank;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.controls.DriverControls;
import frc.robot.controls.DriverControls.Layout;
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

  @org.junit.jupiter.params.ParameterizedTest
  @org.junit.jupiter.params.provider.CsvSource({
    "Xbox Controller, 4, true",
    "Wireless Controller, 2, false",
    "Logitech Dual Action, 2, false",
    "Logitech Dual Action, 4, true"
  })
  void arcadeControlsAndModeTransitions(String name, int rightXAxis, boolean useXbox) {
    var controllerSim = new edu.wpi.first.wpilibj.simulation.GenericHIDSim(0);
    controllerSim.setAxisCount(name.contains("Dual Action") && !useXbox ? 4 : 6);
    controllerSim.setButtonCount(14);
    controllerSim.setName(name);
    DriverStationSim.setJoystickIsXbox(0, useXbox);
    try (var vision = new org.mecorobotics.gamepiecevision.GamePieceVisionClient("teleop-test")) {
      var command =
          new frc.robot.commands.drive.TankTeleopCommand(
              drive, new frc.robot.controls.DriverControls(0, true, true), vision);
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
      controllerSim.setRawAxis(rightXAxis, 1);
      controllerSim.notifyNewData();
      command.execute();
      assertEquals(12, io.left);
      assertEquals(-12, io.right);

      controllerSim.setRawAxis(rightXAxis, -1);
      controllerSim.notifyNewData();
      command.execute();
      assertEquals(-12, io.left);
      assertEquals(12, io.right);
      controllerSim.setRawAxis(rightXAxis, 0.01);
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

  @org.junit.jupiter.params.ParameterizedTest
  @org.junit.jupiter.params.provider.EnumSource(Layout.class)
  void controllerActionMappings(Layout layout) {
    var controls = new DriverControls(0, false, layout);
    var sim = new edu.wpi.first.wpilibj.simulation.GenericHIDSim(0);
    sim.setAxisCount(6);
    sim.setButtonCount(14);
    sim.notifyNewData();
    assertFalse(controls.isIntakeRequested());
    assertFalse(controls.isShootRequested());
    assertFalse(controls.isVisionAssistRequested());
    sim.setRawButton(5, true);
    sim.notifyNewData();
    assertTrue(controls.isIntakeRequested());
    assertFalse(controls.isShootRequested());
    sim.setRawButton(5, false);
    sim.setRawButton(6, true);
    sim.notifyNewData();
    assertFalse(controls.isIntakeRequested());
    assertTrue(controls.isShootRequested());
    if (layout == Layout.XBOX) {
      sim.setRawAxis(3, 0.5);
      sim.notifyNewData();
      assertFalse(controls.isVisionAssistRequested());
      sim.setRawAxis(3, 0.6);
    } else {
      sim.setRawButton(8, true);
    }
    sim.notifyNewData();
    assertTrue(controls.isVisionAssistRequested());
    sim.setRawAxis(3, 0);
    sim.setRawButton(8, false);
    sim.setRawButton(6, false);
    sim.notifyNewData();
    assertFalse(controls.isVisionAssistRequested());
    assertFalse(controls.isShootRequested());
  }

  @Test
  void logitechDetectionOverridesAndDisabledDiagnostics() {
    var sim = new edu.wpi.first.wpilibj.simulation.GenericHIDSim(0);
    sim.setAxisCount(4);
    sim.setButtonCount(12);
    sim.setName("Logitech Dual Action USB");
    sim.setRawAxis(1, -.75);
    sim.setRawAxis(2, .6);
    sim.setRawAxis(3, -.9); // Right-stick vertical must not turn the robot.
    DriverStationSim.setEnabled(false);
    sim.notifyNewData();
    var controls = new DriverControls(0, true, Layout.XBOX);
    assertEquals(Layout.LOGITECH_DUAL_ACTION, controls.layout());
    assertEquals(.6, controls.getRightX(), 1e-6);
    controls.publishDiagnostics();
    assertEquals(
        "Logitech Dual Action (auto)", SmartDashboard.getString("Drive/ControllerLayout", ""));
    assertEquals(4, SmartDashboard.getNumber("Drive/ControllerAxisCount", -1));
    assertEquals(.6, SmartDashboard.getNumber("Drive/RawAxis2", 0), 1e-6);
    assertEquals(0, SmartDashboard.getNumber("Drive/RawAxis4", -1));
    assertEquals(0, io.left);
    assertEquals(0, io.right);

    // A manual selection works even if the reported Xbox flag is wrong.
    DriverStationSim.setJoystickIsXbox(0, true);
    sim.notifyNewData();
    assertEquals(Layout.XBOX, controls.layout());
    var manual = new DriverControls(0, false, Layout.LOGITECH_DUAL_ACTION);
    assertEquals(.6, manual.getRightX(), 1e-6);
    assertEquals("Logitech Dual Action (manual)", manual.layoutName());

    DriverStationSim.setJoystickIsXbox(0, false);
    sim.setName("");
    sim.notifyNewData();
    assertEquals(Layout.XBOX, controls.layout());
    assertEquals(
        Layout.LOGITECH_DUAL_ACTION,
        new DriverControls(0, true, Layout.LOGITECH_DUAL_ACTION).layout());

    sim.setAxisCount(0);
    sim.setButtonCount(0);
    sim.setPOVCount(0);
    sim.notifyNewData();
    controls.publishDiagnostics();
    assertFalse(SmartDashboard.getBoolean("Drive/ControllerConnected", true));
    assertEquals(0, SmartDashboard.getNumber("Drive/RawAxis2", -1));
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
    assertTrue(stoppedX <= frc.robot.sim.fuel.FuelWorldSim.HUB_X - .92);
    assertTrue(stoppedX > frc.robot.sim.fuel.FuelWorldSim.START.getX());
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
