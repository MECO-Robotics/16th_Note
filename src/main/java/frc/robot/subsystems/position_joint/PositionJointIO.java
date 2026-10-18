package frc.robot.subsystems.position_joint;

import frc.robot.constants.Constants;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;
import org.littletonrobotics.junction.AutoLog;

/** Rebuilt position-joint contract, restricted to this robot's single SPARK MAX joint. */
public interface PositionJointIO {
  @AutoLog
  class PositionJointIOInputs {
    public double outputPosition;
    public double velocity;
    public double appliedVolts;
    public double currentAmps;
    public boolean connected;
    public boolean referenced;
    public String configurationStatus = "Not configured";
    public String controlStatus = "Not commanded";
    public String controllerMode = "Unknown";
    public double controllerSetpoint;
    public double profilePosition;
    public double profileVelocity;
    public boolean forwardLimitPressed;
    public boolean reverseLimitPressed;
    public boolean forwardSoftLimitReached;
    public boolean reverseSoftLimitReached;
    public int faultBits;
    public String controllerConfiguration = "Not read";
  }

  static PositionJointIO fromSparkMax(
      String name, PositionJointHardwareConfig config, double maxVolts) {
    if (config.canIds().length != 1
        || config.reversed().length != 1
        || config.encoderType()
            != frc.robot.constants.types.PositionJointConstants.EncoderType.INTERNAL
        || config.mechanismType()
            != frc.robot.constants.types.PositionJointConstants.MechanismType.ROTATIONAL)
      throw new IllegalArgumentException(
          "This robot supports single-motor internal-encoder rotational joints");
    return switch (Constants.currentMode) {
      case REAL -> new PositionJointIOSparkMax(name, config, maxVolts);
      case SIM -> new PositionJointIOSim(name, config, maxVolts);
      case REPLAY -> new PositionJointIO() {
        public String getName() {
          return name;
        }
      };
    };
  }

  default void updateInputs(PositionJointIOInputs inputs) {}

  default void setGains(PositionJointGains gains) {}

  default void setPosition(double position, double velocity) {}

  default void setVoltage(double volts) {}

  default void resetPosition() {}

  default String getName() {
    return "PositionJoint";
  }
}
