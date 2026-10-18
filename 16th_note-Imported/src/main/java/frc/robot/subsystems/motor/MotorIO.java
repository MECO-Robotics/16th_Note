package frc.robot.subsystems.motor;

import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
  @AutoLog
  class MotorIOInputs {
    public double positionRotations;
    public double velocityRpm;
    public double appliedVolts;
    public double currentAmps;
    public boolean connected;
    public boolean resetDetected;
  }

  static MotorIO create(int canId, double reduction, boolean brake, double inertia) {
    return create(canId, reduction, brake, inertia, false);
  }

  static MotorIO create(
      int canId, double reduction, boolean brake, double inertia, boolean inverted) {
    return switch (Constants.currentMode) {
      case REAL -> new MotorIOSparkMax(canId, reduction, brake, inverted);
      case SIM -> new MotorIOSim(reduction, inertia);
      case REPLAY -> new MotorIO() {};
    };
  }

  default void updateInputs(MotorIOInputs inputs) {}

  default void setVoltage(double volts) {}
}
