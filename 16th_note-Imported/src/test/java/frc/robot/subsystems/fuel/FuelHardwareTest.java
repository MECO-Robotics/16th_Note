package frc.robot.subsystems.fuel;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.constants.fuel.FuelConfiguration;
import frc.robot.constants.fuel.FuelConfiguration.*;
import frc.robot.constants.fuel.FuelConstants;
import java.util.EnumMap;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

class FuelHardwareTest {
  static class Motor implements FuelMotor {
    boolean configOk = true,
        writeOk = true,
        clearOk = true,
        limitOk = true,
        readOk = true,
        reset,
        brownout;
    boolean ignoreZero;
    int faults, configs;
    double rotations, rpm, volts, target, min, max;
    boolean minOn, maxOn;
    Integer leader;

    @Override
    public boolean configure(Device d, Integer leader, boolean inverted) {
      configs++;
      this.leader = leader;
      return configOk;
    }

    @Override
    public Sample read() {
      return new Sample(readOk, rotations, rpm, 0, volts, 25, reset, brownout, faults, volts / 12);
    }

    @Override
    public boolean clearFaults() {
      return clearOk;
    }

    @Override
    public boolean zeroEncoder() {
      if (writeOk && !ignoreZero) rotations = 0;
      return writeOk;
    }

    @Override
    public boolean softLimits(double min, double max, boolean minOn, boolean maxOn) {
      this.min = min;
      this.max = max;
      this.minOn = minOn;
      this.maxOn = maxOn;
      return limitOk;
    }

    @Override
    public void voltage(double volts) {
      this.volts = volts;
    }

    @Override
    public void velocity(double rpm) {
      target = rpm;
    }

    @Override
    public void stop() {
      volts = 0;
      target = 0;
    }
  }

  private final EnumMap<Role, Motor> motors = new EnumMap<>(Role.class);
  private final FuelIO.FuelIOInputs inputs = new FuelIO.FuelIOInputs();
  private FuelMode mode = FuelMode.DISABLED;
  private boolean rioBrownout;
  private double now;

  private FuelIOSparkMax create(FuelConfiguration config) {
    return new FuelIOSparkMax(
        config,
        r -> motors.computeIfAbsent(r, k -> new Motor()),
        () -> mode,
        () -> rioBrownout,
        () -> now);
  }

  private FuelConfiguration config() {
    return FuelConfiguration.simulation(FuelConstants.SIMULATION);
  }

  private void reference(FuelIOSparkMax io) {
    io.updateInputs(inputs);
    assertTrue(io.confirmUpperReference());
    io.updateInputs(inputs);
    assertTrue(inputs.referenced);
  }

  private FuelSequencer.Output shot() {
    return new FuelSequencer.Output(true, 20, 3000, 3000, .5, .35);
  }

  private void assertStopped() {
    for (Motor m : motors.values()) {
      assertEquals(0, m.volts);
      assertEquals(0, m.target);
    }
  }

  @Test
  void missingRealValuesAllocateNoMotorsAndReportEachRole() {
    var c = FuelConfiguration.unconfigured();
    var io = create(c);
    io.updateInputs(inputs);
    assertTrue(motors.isEmpty());
    assertFalse(inputs.available);
    assertFalse(inputs.referenced);
    for (Role r : Role.values()) assertTrue(inputs.configurationStatus.contains(r.name()));
    assertFalse(c.automaticIssues().isEmpty());
    assertFalse(c.autoIssues().isEmpty());
  }

  @Test
  void badLimitsAndGainsCannotEnableConfiguration() {
    var d = new Device(0, false, false, 0, Double.NaN, 0, -1, 0);
    assertEquals(5, d.issues().size());
    assertFalse(new Auto(true, 0, -1, -1, 1, 1, 1, 16).issues().isEmpty());
    var invalid =
        new FuelConfiguration(
            config().devices(),
            Coupling.UNCONFIRMED,
            false,
            false,
            Pivot.missing(),
            null,
            Auto.missing());
    assertTrue(invalid.automaticIssues().stream().anyMatch(s -> s.contains("coupling")));
  }

  @Test
  void configurationFailureBlocksOnlyThatDeviceForCommissioning() {
    Motor failed = new Motor();
    failed.configOk = false;
    motors.put(Role.INDEXER_SECONDARY, failed);
    var io = create(config());
    io.updateInputs(inputs);
    assertFalse(inputs.available);
    assertFalse(io.commissioningReady(FuelCommissioning.Selection.INDEXER_PAIR));
    mode = FuelMode.TEST;
    io.commissioning(FuelCommissioning.Selection.INDEXER_PAIR, 100);
    assertEquals(0, motors.get(Role.INDEXER_PRIMARY).volts);
    io.apply(shot());
    assertStopped();
  }

  @Test
  void intakeAndFeedIntentsDriveBothSharedShaftMotorsTogether() {
    var io = create(config());
    reference(io);
    mode = FuelMode.TELEOP;
    io.apply(new FuelSequencer.Output(true, 20, 0, 0, .5, 0));
    assertEquals(6, motors.get(Role.INDEXER_PRIMARY).volts);
    assertEquals(6, motors.get(Role.INDEXER_SECONDARY).volts);
    io.apply(new FuelSequencer.Output(true, 20, 3000, 3000, 0, .35));
    assertEquals(4.2, motors.get(Role.INDEXER_PRIMARY).volts, 1e-9);
    assertEquals(4.2, motors.get(Role.INDEXER_SECONDARY).volts, 1e-9);
  }

  @Test
  void commissioningRunsOnlyTheHealthySharedIndexerPair() {
    var io = create(config());
    io.updateInputs(inputs);
    mode = FuelMode.TEST;
    io.commissioning(FuelCommissioning.Selection.INDEXER_PAIR, 100);
    assertEquals(1, motors.get(Role.INDEXER_PRIMARY).volts);
    assertEquals(1, motors.get(Role.INDEXER_SECONDARY).volts);
  }

  @Test
  void upperReferenceIsDisabledOnlyAndOneMotorTurnIs18Degrees() {
    var io = create(config());
    io.updateInputs(inputs);
    mode = FuelMode.TEST;
    assertFalse(io.confirmUpperReference());
    mode = FuelMode.DISABLED;
    reference(io);
    motors.get(Role.PIVOT).rotations = 1;
    io.updateInputs(inputs);
    assertEquals(18, inputs.pivotDegrees, 1e-9);
    assertEquals(0, motors.get(Role.PIVOT).min);
    assertEquals(160.0 / 18, motors.get(Role.PIVOT).max, 1e-9);
    io.stop();
    io.updateInputs(inputs);
    assertTrue(inputs.referenced); // ordinary disabled stop preserves reference
  }

  @Test
  void failedZeroWriteAndReadbackCannotEstablishReference() {
    var io = create(config());
    io.updateInputs(inputs);
    motors.get(Role.PIVOT).writeOk = false;
    assertFalse(io.confirmUpperReference());
    io.updateInputs(inputs);
    assertFalse(inputs.referenced);
    Motor p = motors.get(Role.PIVOT);
    p.writeOk = true;
    p.ignoreZero = true;
    p.rotations = 2;
    assertTrue(io.confirmUpperReference());
    now = .6;
    io.updateInputs(inputs);
    assertFalse(inputs.referenced);
    assertTrue(inputs.referenceStatus.contains("timed out"));
  }

  @Test
  void lateZeroReadbackCannotEstablishReference() {
    var io = create(config());
    io.updateInputs(inputs);
    assertTrue(io.confirmUpperReference());
    now = .6;
    io.updateInputs(inputs);
    assertFalse(inputs.referenced);
    assertTrue(inputs.referenceStatus.contains("timed out"));
  }

  @Test
  void failedRetryCancelsEarlierPendingReference() {
    var io = create(config());
    io.updateInputs(inputs);
    assertTrue(io.confirmUpperReference());
    motors.get(Role.PIVOT).writeOk = false;
    assertFalse(io.confirmUpperReference());
    io.updateInputs(inputs);
    assertFalse(inputs.referenced);
    assertTrue(inputs.referenceStatus.contains("write failed"));
  }

  @Test
  void failedSoftLimitWriteKeepsPivotUnavailable() {
    var io = create(config());
    io.updateInputs(inputs);
    motors.get(Role.PIVOT).limitOk = false;
    assertTrue(io.confirmUpperReference());
    io.updateInputs(inputs);
    assertFalse(inputs.referenced);
    assertFalse(inputs.available);
  }

  enum Loss {
    RESET,
    BROWNOUT,
    DISCONNECT,
    RIO_BROWNOUT
  }

  @ParameterizedTest
  @EnumSource(Loss.class)
  void lostContinuityInvalidatesReferenceAndStopsAutomaticOutput(Loss loss) {
    var io = create(config());
    reference(io);
    mode = FuelMode.TELEOP;
    io.apply(shot());
    assertEquals(3000, motors.get(Role.SHOOTER_PRIMARY).target);
    Motor p = motors.get(Role.PIVOT);
    switch (loss) {
      case RESET -> p.reset = true;
      case BROWNOUT -> p.brownout = true;
      case DISCONNECT -> p.readOk = false;
      case RIO_BROWNOUT -> rioBrownout = true;
    }
    io.updateInputs(inputs);
    io.apply(shot());
    assertFalse(inputs.referenced);
    assertStopped();
    p.reset = false;
    p.brownout = false;
    p.readOk = true;
    rioBrownout = false;
    io.updateInputs(inputs);
    assertFalse(inputs.referenced);
  }

  @Test
  void bothShooterArrangementsAndStaleReadsAreEnforced() {
    for (Coupling coupling : new Coupling[] {Coupling.COUPLED_FOLLOWER, Coupling.INDEPENDENT}) {
      mode = FuelMode.DISABLED;
      var c = config();
      c =
          new FuelConfiguration(
              c.devices(), coupling, true, true, c.pivot(), c.sequence(), c.auto());
      var io = create(c);
      reference(io);
      mode = FuelMode.TELEOP;
      io.apply(shot());
      assertEquals(3000, motors.get(Role.SHOOTER_PRIMARY).target);
      if (coupling == Coupling.COUPLED_FOLLOWER) {
        assertEquals(3, motors.get(Role.SHOOTER_SECONDARY).leader);
        assertEquals(0, motors.get(Role.SHOOTER_SECONDARY).target);
      } else {
        assertNull(motors.get(Role.SHOOTER_SECONDARY).leader);
        assertEquals(3000, motors.get(Role.SHOOTER_SECONDARY).target);
      }
      now += .2;
      io.apply(shot());
      assertStopped();
      int configs = motors.get(Role.PIVOT).configs;
      io.updateInputs(inputs);
      io.apply(shot());
      assertEquals(configs, motors.get(Role.PIVOT).configs);
    }
  }

  @Test
  void boundedPivotJogDoesNotNeedAutomaticTravelButRequiresReference() {
    var c = config();
    c =
        new FuelConfiguration(
            c.devices(), c.coupling(), false, false, Pivot.missing(), null, Auto.missing());
    var io = create(c);
    io.updateInputs(inputs);
    mode = FuelMode.TEST;
    io.commissioning(FuelCommissioning.Selection.PIVOT, 1);
    assertStopped();
    mode = FuelMode.DISABLED;
    reference(io);
    assertFalse(motors.get(Role.PIVOT).maxOn);
    mode = FuelMode.TEST;
    io.commissioning(FuelCommissioning.Selection.PIVOT, 10);
    assertEquals(1, motors.get(Role.PIVOT).volts);
    io.apply(shot());
    assertStopped();
  }

  @Test
  void oppositeEncoderSignMapsUpperAndLowerSoftLimitsCorrectly() {
    var c = config();
    var devices = new EnumMap<Role, Device>(Role.class);
    devices.putAll(c.devices());
    Device p = c.device(Role.PIVOT);
    devices.put(
        Role.PIVOT,
        new Device(p.currentAmps(), true, true, -1, p.maxVolts(), p.testVolts(), p.kP(), p.kV()));
    var io =
        create(
            new FuelConfiguration(
                devices, c.coupling(), false, false, c.pivot(), c.sequence(), c.auto()));
    reference(io);
    assertEquals(-160.0 / 18, motors.get(Role.PIVOT).min, 1e-9);
    assertEquals(0, motors.get(Role.PIVOT).max);
    motors.get(Role.PIVOT).rotations = -1;
    io.updateInputs(inputs);
    assertEquals(18, inputs.pivotDegrees);
  }
}
