package frc.robot.subsystems.fuel;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.fuel.FuelConfiguration;
import frc.robot.constants.fuel.FuelConstants;
import frc.robot.sim.fuel.FuelIOSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FuelAngleTuningTest {
  private double now;
  private FuelSystem fuel;
  private FuelAngleTuning tuning;
  private IO io;
  private NetworkTableInstance persistence;
  private FuelAngleStore store;

  private static class IO implements FuelIO {
    FuelSequencer.Output output = FuelSequencer.Output.stopped();
    int configureCalls;
    int referenceWrites;

    @Override
    public void updateInputs(FuelIOInputs inputs) {
      inputs.available = inputs.healthy = inputs.referenced = true;
      inputs.shooterPrimaryConnected =
          inputs.shooterSecondaryConnected = inputs.indexerConnected = true;
      inputs.pivotDegrees = 160;
      inputs.shooterPrimaryMotorRpm = inputs.shooterSecondaryMotorRpm = 3000;
      inputs.ageSeconds = 0;
    }

    @Override
    public void apply(FuelSequencer.Output request) {
      output = request;
    }

    @Override
    public void stop() {
      output = FuelSequencer.Output.stopped();
    }

    @Override
    public boolean configure(FuelConfiguration config) {
      configureCalls++;
      return true;
    }

    @Override
    public void invalidateReference(String reason) {
      referenceWrites++;
    }

    @Override
    public boolean confirmUpperReference() {
      referenceWrites++;
      return true;
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
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    io = new IO();
    var settings = FuelConstants.SIMULATION.withAgitation(true);
    fuel = new FuelSystem(io, FuelConfiguration.simulation(settings), () -> now);
    persistence = NetworkTableInstance.create();
    store = new FuelAngleStore(persistence, "SIM");
    tuning = new FuelAngleTuning(settings, store);
    refresh();
  }

  @AfterEach
  void cleanup() {
    CommandScheduler.getInstance().unregisterSubsystem(fuel);
    persistence.close();
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  private void refresh() {
    tuning.feed.periodic();
    tuning.agitation.periodic();
    tuning.update(fuel);
    fuel.periodic();
  }

  private void edit(double upper, double lower) {
    var nt = NetworkTableInstance.getDefault();
    nt.getEntry("/Tuning/Fuel/FeedDegrees").setDouble(upper);
    nt.getEntry("/Tuning/Fuel/AgitationDegrees").setDouble(lower);
    refresh();
  }

  private void shootFrame() {
    now += .02;
    fuel.request(true, false, true);
    refresh();
    assertNotEquals(FuelSequencer.State.FAULT, fuel.state(), fuel.fault());
  }

  private void awaitState(FuelSequencer.State state) {
    for (int i = 0; i < 300 && fuel.state() != state; i++) shootFrame();
    assertEquals(state, fuel.state());
  }

  @Test
  void restartRestoresLastAcceptedPairRatherThanDefaultsOrRejectedEdits() {
    edit(40, 70);
    edit(100, 70); // Invalid ordering must not replace the saved pair.
    assertArrayEquals(new double[] {40, 70}, store.load());
    CommandScheduler.getInstance().unregisterSubsystem(fuel);
    var defaults = FuelConstants.SIMULATION.withAgitation(true);
    fuel = new FuelSystem(io, FuelConfiguration.simulation(defaults), () -> now);
    tuning = new FuelAngleTuning(defaults, store);
    refresh();
    assertEquals(40, tuning.feed.get());
    assertEquals(70, tuning.agitation.get());
    assertEquals(40, SmartDashboard.getNumber("Fuel/Config/FeedDegrees", -1));
    assertEquals(70, SmartDashboard.getNumber("Fuel/Config/AgitationDegrees", -1));
    assertEquals(0, io.configureCalls);
    assertEquals(0, io.referenceWrites);
  }

  @Test
  void pendingHardwareEditsAreSavedOnlyAfterTheyApply() {
    edit(40, 70);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    edit(50, 80);
    assertArrayEquals(new double[] {40, 70}, store.load());
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    refresh();
    assertArrayEquals(new double[] {50, 80}, store.load());
  }

  @Test
  void savedAnglesAreRevalidatedAndDoNotConfigureMissingHardware() {
    store.save(0, 70);
    tuning = new FuelAngleTuning(FuelConstants.SIMULATION.withAgitation(true), store);
    refresh();
    assertTrue(SmartDashboard.getString("Fuel/AngleTuningStatus", "").startsWith("Rejected"));
    store.save(40, 70);
    fuel.applyConfiguration(FuelConfiguration.unconfigured());
    tuning = new FuelAngleTuning(null, store);
    refresh();
    assertEquals(40, SmartDashboard.getNumber("Fuel/Config/FeedDegrees", -1));
    assertArrayEquals(new double[] {40, 70}, store.load());
    assertTrue(
        SmartDashboard.getString("Fuel/AngleTuningStatus", "")
            .contains("configure fuel sequence first"));
  }

  @Test
  void enabledSimulationRetargetsFeedAndAgitationWithoutRearming() {
    CommandScheduler.getInstance().unregisterSubsystem(fuel);
    var settings = FuelConstants.SIMULATION.withAgitation(true);
    fuel =
        new FuelSystem(
            new FuelIOSim(settings, false), FuelConfiguration.simulation(settings), () -> now);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    fuel.request(true, false, false);
    refresh();
    awaitState(FuelSequencer.State.FEEDING);

    edit(40, 70);
    assertEquals(FuelSequencer.State.FEEDING, fuel.state());
    assertEquals(40, SmartDashboard.getNumber("Fuel/Config/FeedDegrees", -1));
    for (int i = 0; i < 23; i++) shootFrame();
    assertEquals(40, fuel.pivotDegrees(), settings.positionToleranceDegrees());

    awaitState(FuelSequencer.State.AGITATING_DOWN);
    edit(40, 100);
    assertEquals(FuelSequencer.State.AGITATING_DOWN, fuel.state());
    assertEquals(100, SmartDashboard.getNumber("Fuel/Config/AgitationDegrees", -1));
    awaitState(FuelSequencer.State.AGITATING_UP);
    assertEquals(100, fuel.pivotDegrees(), settings.positionToleranceDegrees());
    assertTrue(SmartDashboard.getBoolean("Fuel/Referenced", false));

    edit(50, 100);
    assertEquals(FuelSequencer.State.AGITATING_UP, fuel.state());
    awaitState(FuelSequencer.State.FEEDING);
    assertEquals(50, fuel.pivotDegrees(), settings.positionToleranceDegrees());
  }

  @Test
  void networkEditsChangeActualRaisedTargetWithoutControllerWritesOrReferenceLoss() {
    edit(40, 70);
    assertEquals(40, SmartDashboard.getNumber("Fuel/Config/FeedDegrees", -1));
    assertEquals(70, SmartDashboard.getNumber("Fuel/Config/AgitationDegrees", -1));
    assertTrue(SmartDashboard.getBoolean("Fuel/Referenced", false));
    assertEquals(0, io.configureCalls);
    assertEquals(0, io.referenceWrites);
    assertEquals(0, io.output.indexerDuty());

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    fuel.request(true, false, false);
    fuel.periodic();
    for (int i = 0; i < 20; i++) {
      now += .02;
      fuel.request(true, false, true);
      fuel.periodic();
    }
    assertEquals(FuelSequencer.State.FEEDING, fuel.state());
    assertEquals(40, io.output.pivotDegrees());
    assertEquals(3000, io.output.shooterPrimaryMotorRpm());
  }

  @Test
  void enabledEditsWaitAndInvalidAnglesRetainLastValidTargets() {
    edit(40, 70);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    edit(50, 80);
    assertEquals(40, SmartDashboard.getNumber("Fuel/Config/FeedDegrees", -1));
    assertTrue(SmartDashboard.getString("Fuel/AngleTuningStatus", "").startsWith("Pending"));
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    refresh();
    assertEquals(50, SmartDashboard.getNumber("Fuel/Config/FeedDegrees", -1));
    for (double[] angles :
        new double[][] {{Double.NaN, 80}, {0, 80}, {161, 80}, {50, 40}, {50, 160}}) {
      edit(angles[0], angles[1]);
      assertEquals(50, SmartDashboard.getNumber("Fuel/Config/FeedDegrees", -1));
      assertEquals(80, SmartDashboard.getNumber("Fuel/Config/AgitationDegrees", -1));
      assertTrue(SmartDashboard.getString("Fuel/AngleTuningStatus", "").startsWith("Rejected"));
    }
    assertEquals(0, io.configureCalls);
    assertEquals(0, io.referenceWrites);
  }

  @Test
  void fullConfigurationSyncDoesNotRestoreStaleTuningAndMissingConfigurationIsBlocked() {
    edit(40, 70);
    var settings = FuelConstants.SIMULATION.withIntakeAngles(55, 85).withAgitation(true);
    fuel.applyConfiguration(FuelConfiguration.simulation(settings));
    tuning.synchronize(settings);
    assertArrayEquals(new double[] {55, 85}, store.load());
    tuning.update(fuel); // Cached values from before the full apply must be ignored.
    fuel.periodic();
    assertTrue(
        SmartDashboard.getString("Fuel/ActiveConfiguration", "").contains("feedDegrees=55.0"));
    refresh();
    assertEquals(55, tuning.feed.get());
    fuel.applyConfiguration(FuelConfiguration.unconfigured());
    tuning.synchronize(null);
    assertArrayEquals(new double[] {55, 85}, store.load());
    assertFalse(fuel.tuneIntakeAngles(40, 70));
    fuel.periodic();
    assertTrue(
        SmartDashboard.getString("Fuel/AngleTuningStatus", "")
            .contains("configure fuel sequence first"));
  }
}
