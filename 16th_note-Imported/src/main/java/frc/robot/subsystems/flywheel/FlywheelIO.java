package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.Constants;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;
import org.littletonrobotics.junction.AutoLog;

/** Rebuilt flywheel contract: mechanism rotations and rotations/second. */
public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public double velocity;
    public double position;
    public double desiredVelocity;
    public double appliedVolts;
    public double currentAmps;
    public boolean connected;
  }

  static FlywheelIO fromSparkMax(String name, FlywheelHardwareConfig config) {
    if (config.canIds().length != 1 || config.reversed().length != 1)
      throw new IllegalArgumentException("This robot supports single-motor flywheels");
    return switch (Constants.currentMode) {
      case REAL -> new FlywheelIOSparkMax(name, config);
      case SIM -> new FlywheelIOSimSparkMax(name, config, DCMotor.getNEO(1));
      case REPLAY -> new FlywheelIO() {
        public String getName() {
          return name;
        }
      };
    };
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void setGains(FlywheelGains gains) {}

  default void setVelocity(double velocity) {}

  default void setVoltage(double volts) {}

  String getName();
}
