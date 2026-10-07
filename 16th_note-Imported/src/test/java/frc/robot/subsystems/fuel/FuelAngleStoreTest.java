package frc.robot.subsystems.fuel;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class FuelAngleStoreTest {
  @TempDir Path directory;

  @Test
  void acceptedPairSurvivesNetworkTablesServerRestartWithoutDashboard() throws Exception {
    Path file = directory.resolve("networktables.json");
    try (var first = NetworkTableInstance.create()) {
      // Ephemeral ports and a temporary file cannot interfere with the user's simulator.
      first.startServer(file.toString(), "127.0.0.1", 0, 0);
      var store = new FuelAngleStore(first, "SIM");
      store.save(47, 93);
      long deadline = System.nanoTime() + 5_000_000_000L;
      while ((!Files.exists(file) || !Files.readString(file).contains("93"))
          && System.nanoTime() < deadline) Thread.sleep(20);
      assertTrue(Files.exists(file), "NetworkTables must save its persistent file");
      assertTrue(Files.readString(file).contains("93"));
      first.stopServer();
    }
    try (var restarted = NetworkTableInstance.create()) {
      restarted.startServer(file.toString(), "127.0.0.1", 0, 0);
      var store = new FuelAngleStore(restarted, "SIM");
      double[] expected = {47, 93};
      long deadline = System.nanoTime() + 5_000_000_000L;
      while (!Arrays.equals(expected, store.load()) && System.nanoTime() < deadline)
        Thread.sleep(20);
      assertArrayEquals(expected, store.load());
      assertEquals(0, new FuelAngleStore(restarted, "REAL").load().length);
      restarted.stopServer();
    }
  }
}
