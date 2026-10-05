package frc.robot.subsystems.fuel;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controls.DriverControls;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class FuelSystemTest {
  private static class FakeIO implements FuelIO {
    FuelSequencer.Output output = FuelSequencer.Output.stopped();
    double degrees = 70;
    double rpm;
    double secondaryRpm;
    boolean primaryConnected = true;
    boolean secondaryConnected = true;
    boolean indexerConnected = true;

    @Override
    public void updateInputs(FuelIOInputs inputs) {
      inputs.available = true;
      inputs.healthy = true;
      inputs.referenced = true;
      inputs.pivotDegrees = degrees;
      inputs.shooterPrimaryMotorRpm = rpm;
      inputs.shooterSecondaryMotorRpm = secondaryRpm;
      inputs.shooterPrimaryConnected = primaryConnected;
      inputs.shooterSecondaryConnected = secondaryConnected;
      inputs.indexerConnected = indexerConnected;
      inputs.ageSeconds = 0;
    }

    @Override
    public void apply(FuelSequencer.Output output) {
      this.output = output;
    }
  }

  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private FakeIO io;
  private FuelSystem fuel;
  private double now;
  private SubsystemBase other;

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void setup() {
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    io = new FakeIO();
    fuel = new FuelSystem(io, FuelTestSettings.EXAMPLE, () -> now);
    other = new SubsystemBase() {};
  }

  @AfterEach
  void cleanup() {
    scheduler.cancelAll();
    scheduler.unregisterSubsystem(fuel, other);
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  private void loop() {
    now += .02;
    scheduler.run();
  }

  @ParameterizedTest
  @ValueSource(booleans = {true, false})
  void bothControllerLayoutsRunSequenceWithoutTakingDrivetrain(boolean xbox) {
    var controls = new DriverControls(0, xbox);
    var sim = new GenericHIDSim(0);
    sim.setAxisCount(6);
    sim.setButtonCount(14);
    sim.notifyNewData();
    assertTrue(controls.isConnected());
    var command = fuel.teleopCommand(controls);
    assertEquals(1, command.getRequirements().size());
    assertTrue(command.getRequirements().contains(fuel));
    fuel.setDefaultCommand(command);
    var driving = other.run(() -> {});
    scheduler.schedule(driving);
    for (int i = 0; i < 4; i++) loop();
    assertEquals(FuelSequencer.State.IDLE, fuel.state());
    sim.setRawButton(5, true); // LB and L1 share this button index; stick mappings differ.
    sim.notifyNewData();
    loop();
    loop();
    assertEquals(0, io.output.pivotDegrees());
    io.degrees = 0;
    loop();
    assertTrue(io.output.intakeDuty() > 0);
    sim.setRawButton(5, false);
    sim.notifyNewData();
    loop();
    loop();
    assertEquals(FuelSequencer.State.DEPLOYED, fuel.state());
    assertEquals(0, io.output.intakeDuty());
    sim.setRawButton(6, true);
    sim.notifyNewData();
    loop();
    loop();
    assertEquals(FuelSequencer.State.SPINNING_UP, fuel.state());
    assertEquals(3000, io.output.shooterPrimaryMotorRpm());
    assertEquals(0, io.output.indexerDuty());
    assertEquals(0, io.output.pivotDegrees());
    io.rpm = 3000;
    io.secondaryRpm = 3000;
    for (int i = 0; i < 15; i++) loop();
    assertEquals(FuelSequencer.State.FEEDING, fuel.state());
    assertEquals(70, io.output.pivotDegrees());
    assertEquals(3000, io.output.shooterSecondaryMotorRpm());
    assertTrue(io.output.indexerDuty() > 0);
    assertTrue(scheduler.isScheduled(driving));
    scheduler.cancel(command);
    assertEquals(FuelSequencer.Output.stopped(), io.output);
    // A held bumper must not restart the default command after interruption.
    for (int i = 0; i < 4; i++) loop();
    assertEquals(FuelSequencer.Output.stopped(), io.output);
    assertEquals(FuelSequencer.State.WAIT_FOR_RELEASE, fuel.state());
  }

  @Test
  void missingRequestWatchdogStopsOutputsAndRequiresRearm() {
    fuel.request(true, false, false);
    fuel.periodic();
    now = .02;
    fuel.request(true, true, false);
    fuel.periodic();
    assertTrue(io.output.pivotEnabled());
    now = .2;
    fuel.periodic();
    assertEquals(FuelSequencer.Output.stopped(), io.output);
    now = .22;
    fuel.request(true, true, false);
    fuel.periodic();
    assertEquals(FuelSequencer.Output.stopped(), io.output);
  }

  @Test
  void disableAutonomousAndTestCannotRunMechanisms() {
    for (int mode = 0; mode < 3; mode++) {
      DriverStationSim.setEnabled(mode != 0);
      DriverStationSim.setAutonomous(mode == 1);
      DriverStationSim.setTest(mode == 2);
      DriverStationSim.notifyNewData();
      fuel.request(true, false, false);
      fuel.periodic();
      fuel.request(true, true, true);
      fuel.periodic();
      assertEquals(FuelSequencer.Output.stopped(), io.output);
    }
  }

  @ParameterizedTest
  @ValueSource(ints = {0, 1, 2})
  void losingEitherShooterOrIndexerConnectionStopsEveryOutput(int device) {
    fuel.request(true, false, false);
    fuel.periodic();
    io.rpm = 3000;
    io.secondaryRpm = 3000;
    for (int i = 0; i < 20; i++) {
      now += .02;
      fuel.request(true, false, true);
      fuel.periodic();
    }
    assertTrue(io.output.indexerDuty() > 0);
    if (device == 0) io.primaryConnected = false;
    if (device == 1) io.secondaryConnected = false;
    if (device == 2) io.indexerConnected = false;
    now += .02;
    fuel.request(true, false, true);
    fuel.periodic();
    assertEquals(FuelSequencer.Output.stopped(), io.output);
    assertEquals(FuelSequencer.State.FAULT, fuel.state());
  }

  @Test
  void unconfiguredIOIsUnavailableAndCannotMove() {
    scheduler.unregisterSubsystem(fuel);
    fuel = new FuelSystem(new FuelIO() {}, FuelTestSettings.EXAMPLE, () -> now);
    fuel.request(true, false, false);
    fuel.periodic();
    fuel.request(true, true, true);
    fuel.periodic();
    assertEquals(FuelSequencer.State.UNAVAILABLE, fuel.state());
  }
}
