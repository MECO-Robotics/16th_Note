package frc.robot.subsystems.tank;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.*;
import frc.robot.sim.fuel.FuelWorldSim;
import frc.robot.sim.tank.PracticeField;
import org.junit.jupiter.api.Test;

class PracticeFieldTest {
  @Test
  void sweepCannotTunnelThroughEitherHub() {
    for (double x : new double[] {FuelWorldSim.HUB_X, FuelWorldSim.RED_HUB_X}) {
      var start = new Pose2d(x - 2, FuelWorldSim.HUB_Y, new Rotation2d());
      var result =
          PracticeField.constrain(start, new Pose2d(x + 2, FuelWorldSim.HUB_Y, new Rotation2d()));
      assertTrue(result.getX() <= x - .92);
      assertFalse(PracticeField.blocked(result.getX(), result.getY()));
    }
  }

  @Test
  void wallsStopTranslationButAllowRotationAndBackingAway() {
    var start = new Pose2d(.4, 2, new Rotation2d());
    var hit = PracticeField.constrain(start, new Pose2d(-2, 2, Rotation2d.fromDegrees(90)));
    assertTrue(hit.getX() >= .32);
    assertEquals(90, hit.getRotation().getDegrees(), 1e-9);
    var away = new Pose2d(1, 2, new Rotation2d());
    assertEquals(away, PracticeField.constrain(hit, away));
  }

  @Test
  void openSpaceIsUnchangedAndAllWallsAreBlocked() {
    var next = new Pose2d(2, 2, new Rotation2d());
    assertEquals(next, PracticeField.constrain(new Pose2d(1, 1, new Rotation2d()), next));
    assertTrue(PracticeField.blocked(.1, 2));
    assertTrue(PracticeField.blocked(16.5, 2));
    assertTrue(PracticeField.blocked(2, .1));
    assertTrue(PracticeField.blocked(2, 8));
  }
}
