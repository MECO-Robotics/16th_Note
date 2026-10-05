package frc.robot.subsystems.fuel;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.sim.fuel.FuelVisualization;
import org.junit.jupiter.api.Test;

class FuelVisualizationTest {
  @Test
  void deployedMatchesCadAndRaisingKeepsPivotFixed() {
    var down = FuelVisualization.componentPoses(160);
    var up = FuelVisualization.componentPoses(0);
    assertEquals(1, down.length);
    assertEquals(0, down[0].getRotation().getAngle(), 1e-9);
    assertEquals(down[0].getTranslation(), up[0].getTranslation());
    assertEquals(new Rotation3d(0, Math.toRadians(-160), 0), up[0].getRotation());
    // Outer roller center relative to the pivot in the exported lowered pose.
    var roller = new Translation3d(0.3003543758470656, 0, -0.211709);
    assertTrue(roller.rotateBy(up[0].getRotation()).getZ() > roller.getZ());
  }
}
