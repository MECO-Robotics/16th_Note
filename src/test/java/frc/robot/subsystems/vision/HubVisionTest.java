package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class HubVisionTest {
  @Test
  void recognizesOnly2026HubTags() {
    for (int id : new int[] {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27}) {
      assertTrue(HubVision.isHubTag(id), "Expected hub tag " + id);
    }
    for (int id : new int[] {-1, 1, 6, 7, 12, 13, 17, 22, 23, 28, 32}) {
      assertFalse(HubVision.isHubTag(id), "Expected non-hub tag " + id);
    }
  }
}
