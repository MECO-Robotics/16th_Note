package frc.robot.subsystems.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Open-loop intake/indexer roller; closed-loop mechanisms use Flywheel or PositionJoint. */
public final class Motor extends SubsystemBase {
  private final MotorIO io;
  private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
  private final double maxVolts;
  private double targetVolts;

  public Motor(String name, MotorIO io, double maxVolts) {
    super(name);
    this.io = io;
    this.maxVolts = maxVolts;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    boolean enabledForControl =
        DriverStation.isAutonomousEnabled()
            || (DriverStation.isTeleopEnabled() && DriverStation.isJoystickConnected(0));
    if (!enabledForControl || !inputs.connected || !Double.isFinite(targetVolts)) stop();
    io.setVoltage(MathUtil.clamp(targetVolts, -maxVolts, maxVolts));
    Logger.recordOutput(getName() + "/CommandedVolts", targetVolts);
  }

  public Command voltageCommand(DoubleSupplier volts) {
    return runEnd(() -> targetVolts = volts.getAsDouble(), this::stop);
  }

  public void stop() {
    targetVolts = 0;
    io.setVoltage(0);
  }
}
