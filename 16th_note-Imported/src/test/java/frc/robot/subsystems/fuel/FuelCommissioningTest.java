package frc.robot.subsystems.fuel;

import static frc.robot.subsystems.fuel.FuelCommissioning.Selection.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class FuelCommissioningTest {
  @Test
  void deadmanSelectionAndOutputChangesRequireRelease() {
    var c = new FuelCommissioning();
    assertEquals(0, c.update(0, true, true, true, ROLLERS, 1, true));
    c.update(.02, true, true, false, ROLLERS, 1, true);
    assertEquals(1, c.update(.04, true, true, true, ROLLERS, 1, true));
    assertEquals(0, c.update(.06, true, true, false, ROLLERS, 1, true));
    assertEquals(1, c.update(.08, true, true, true, ROLLERS, 1, true));
    assertEquals(0, c.update(.1, true, true, true, INDEXER, 1, true));
    c.update(.12, true, true, false, INDEXER, 1, true);
    assertEquals(1, c.update(.14, true, true, true, INDEXER, 1, true));
    assertEquals(0, c.update(.16, true, true, true, INDEXER, 2, true));
  }

  @Test
  void timeoutsConnectionsModesAndFaultsStopBursts() {
    var c = new FuelCommissioning();
    c.update(0, true, true, false, PIVOT, 1, true);
    assertEquals(1, c.update(.02, true, true, true, PIVOT, 1, true));
    assertEquals(0, c.update(.28, true, true, true, PIVOT, 1, true));
    assertEquals(0, c.update(.3, true, true, true, PIVOT, 1, true));
    c.update(.32, true, true, false, PIVOT, 1, true);
    assertEquals(1, c.update(.34, true, true, true, PIVOT, 1, true));
    assertEquals(0, c.update(.36, true, false, true, PIVOT, 1, true));
    assertEquals(0, c.update(.38, true, true, true, PIVOT, 1, true));
    c.update(.4, true, true, false, PIVOT, 1, true);
    assertEquals(0, c.update(.42, false, true, true, PIVOT, 1, true));
    c.update(.44, true, true, false, PIVOT, 1, true);
    assertEquals(0, c.update(.46, true, true, true, PIVOT, 1, false));
    assertEquals(0, c.update(.48, true, true, true, PIVOT, 1, true));
  }
}
