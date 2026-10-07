package frc.robot.subsystems.fuel;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Arrays;

/** Stores the last accepted pair together in WPILib's persistent NetworkTables file. */
final class FuelAngleStore {
  private final NetworkTableEntry entry;

  FuelAngleStore(NetworkTableInstance instance, String mode) {
    entry = instance.getEntry("/Preferences/16thNote/Fuel/" + mode + "/IntakeAngles");
  }

  double[] load() {
    double[] values = entry.getDoubleArray(new double[0]);
    return values.length == 2 && Double.isFinite(values[0]) && Double.isFinite(values[1])
        ? values
        : new double[0];
  }

  void save(double feed, double agitation) {
    double[] values = {feed, agitation};
    if (!Arrays.equals(values, load())) entry.setDoubleArray(values);
    entry.setPersistent();
  }
}
