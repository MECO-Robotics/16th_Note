package frc.robot.commands.shooter;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class DistanceBasedShooterControllerTest {
  @Test
  void returnsEachMeasuredCalibrationPoint() {
    assertEquals(40.0, calculate(1.0).targetRps(), 1e-9);
    assertEquals(50.0, calculate(2.0).targetRps(), 1e-9);
    assertEquals(70.0, calculate(4.0).targetRps(), 1e-9);
  }

  @Test
  void interpolatesBetweenMeasuredPoints() {
    assertEquals(45.0, calculate(1.5).targetRps(), 1e-9);
    assertEquals(60.0, calculate(3.0).targetRps(), 1e-9);
  }

  @Test
  void rejectsDistancesOutsideTestedRange() {
    assertFalse(calculate(0.99).inRange());
    assertFalse(calculate(4.01).inRange());
    assertEquals(0.0, calculate(4.01).targetRps(), 1e-9);
  }

  @Test
  void rejectsUnconfiguredOrInvalidCalibration() {
    var unconfigured =
        DistanceBasedShooterController.calculate(2.0, 0.0, 40.0, 0.0, 50.0, 0.0, 70.0, 100.0);
    var unordered =
        DistanceBasedShooterController.calculate(2.0, 1.0, 40.0, 0.5, 50.0, 4.0, 70.0, 100.0);
    var impossibleSpeed =
        DistanceBasedShooterController.calculate(2.0, 1.0, 40.0, 2.0, 101.0, 4.0, 70.0, 100.0);
    assertFalse(unconfigured.configured());
    assertFalse(unordered.configured());
    assertFalse(impossibleSpeed.configured());
    assertEquals(0.0, unconfigured.targetRps(), 1e-9);
  }

  private static DistanceBasedShooterController.Output calculate(double distanceMeters) {
    return DistanceBasedShooterController.calculate(
        distanceMeters, 1.0, 40.0, 2.0, 50.0, 4.0, 70.0, 100.0);
  }
}
