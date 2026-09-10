package frc.robot.subsystems.tank;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.constants.Constants;
import frc.robot.sim.replay.tank.TankDriveIOReplay;
import frc.robot.sim.tank.TankDriveIOSim;
import org.littletonrobotics.junction.AutoLog;

/**
 * Positive voltage moves that side forward. Real wheel-position and heading feedback are not
 * configured.
 */
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

  /** Creates the real, simulated, or replay IO for a SPARK MAX tank drivetrain. */
  static TankDriveIO fromSparkMax(MotorType motorType) {
    return switch (Constants.currentMode) {
      case REAL -> new TankDriveIOSparkMax(motorType);
      case SIM -> new TankDriveIOSim();
      case REPLAY -> new TankDriveIOReplay();
    };
  }

  default void updateInputs(TankDriveIOInputs inputs) {}

  default void setVoltage(double leftVolts, double rightVolts) {}

  default void stop() {
    setVoltage(0.0, 0.0);
  }
}
