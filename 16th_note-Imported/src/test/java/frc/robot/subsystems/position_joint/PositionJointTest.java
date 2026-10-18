package frc.robot.subsystems.position_joint;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import org.junit.jupiter.api.Test;

class PositionJointTest {
  @Test
  void startupHomeAndGravityFrameAgreeWithPhysicalAngles() {
    assertTrue(HAL.initialize(500, 0));
    var io =
        new PositionJointIOSim(
            "Pivot", IntakeConstants.PIVOT_CONFIG, IntakeConstants.PIVOT_MAX_VOLTS);
    var inputs = new PositionJointIO.PositionJointIOInputs();
    io.updateInputs(inputs);
    assertEquals(0.25, inputs.outputPosition, 1e-9);
    assertEquals(0, inputs.appliedVolts, 1e-9);
    assertTrue(inputs.referenced);
    var defaults = IntakeConstants.PIVOT_GAINS;
    var gravityOnly =
        new PositionJointGains(
            0,
            0,
            0,
            0,
            0.6,
            0,
            0,
            defaults.kMaxVelo(),
            defaults.kMaxAccel(),
            defaults.kMinPosition(),
            defaults.kMaxPosition(),
            defaults.kTolerance(),
            defaults.kDefaultSetpoint());
    io.setGains(gravityOnly);
    io.setPosition(0.25, 0);
    io.updateInputs(inputs);
    assertEquals(0, inputs.appliedVolts, 1e-9, "No gravity feedforward at vertical home");
    io.setGains(defaults);
    io.setPosition(0, 0);
    for (int tick = 0; tick < 200; tick++) io.updateInputs(inputs);
    assertEquals(0, inputs.outputPosition, 0.01);
    io.setGains(gravityOnly);
    io.updateInputs(inputs);
    assertEquals(0.6, inputs.appliedVolts, 0.001, "Maximum gravity feedforward at horizontal");
  }

  @Test
  void missingReferenceDoesNotGatePositionButDisconnectStopsOutput() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    var joystick = new GenericHIDSim(0);
    joystick.setAxisCount(6);
    joystick.setButtonCount(10);
    joystick.notifyNewData();
    class FakeIO implements PositionJointIO {
      boolean referenced = true;
      boolean connected = true;
      boolean positionCalled;
      double volts;

      public void updateInputs(PositionJointIOInputs inputs) {
        inputs.connected = connected;
        inputs.referenced = referenced;
        inputs.outputPosition = 0.25;
      }

      public void setPosition(double position, double velocity) {
        positionCalled = true;
      }

      public void setVoltage(double voltage) {
        volts = voltage;
      }
    }
    var io = new FakeIO();
    var joint = new PositionJoint(io, IntakeConstants.PIVOT_GAINS);
    try {
      joint.setPosition(0.125);
      joint.periodic();
      assertTrue(io.positionCalled);
      io.positionCalled = false;
      io.referenced = false;
      joint.periodic();
      assertTrue(io.positionCalled, "Reference status is diagnostic, not a motion gate");
      io.positionCalled = false;
      io.connected = false;
      joint.periodic();
      assertFalse(io.positionCalled);
      assertEquals(0, io.volts);
    } finally {
      joint.stop();
      CommandScheduler.getInstance().unregisterSubsystem(joint);
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    }
  }
}
