package frc.robot.subsystems.tank;

import org.littletonrobotics.junction.AutoLog;

/** Positive voltage moves that side forward. Real PWM hardware has no feedback sensors. */
public interface TankDriveIO {
  @AutoLog
  class TankDriveIOInputs {
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
    public boolean hasPositionFeedback = false;
    public double leftPositionMeters = 0.0;
    public double rightPositionMeters = 0.0;
    public double leftVelocityMetersPerSecond = 0.0;
    public double rightVelocityMetersPerSecond = 0.0;
    public double headingRadians = 0.0;
  }

  default void updateInputs(TankDriveIOInputs inputs) {}

  default void setVoltage(double leftVolts, double rightVolts) {}

  default void stop() {
    setVoltage(0.0, 0.0);
  }
}
