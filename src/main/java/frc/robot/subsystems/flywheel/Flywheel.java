package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Rebuilt flywheel gain-update pattern, with controller-side velocity PID on hardware. */
public final class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final LoggedTunableNumber kP, kI, kD, kS, kV, kA, kMaxAccel, kTolerance, kSetpoint;
  private boolean active;
  private boolean gainsValid = true;
  private double velocitySetpoint;

  public Flywheel(FlywheelIO io, FlywheelGains gains) {
    super(io.getName());
    String name = io.getName();
    this.io = io;
    kP = new LoggedTunableNumber(name + "/Gains/kP", gains.kP());
    kI = new LoggedTunableNumber(name + "/Gains/kI", gains.kI());
    kD = new LoggedTunableNumber(name + "/Gains/kD", gains.kD());
    kS = new LoggedTunableNumber(name + "/Gains/kS", gains.kS());
    kV = new LoggedTunableNumber(name + "/Gains/kV", gains.kV());
    kA = new LoggedTunableNumber(name + "/Gains/kA", gains.kA());
    kMaxAccel = new LoggedTunableNumber(name + "/Gains/kMaxAccel", gains.kMaxAccel());
    kTolerance = new LoggedTunableNumber(name + "/Gains/kTolerance", gains.kTolerance());
    kSetpoint = new LoggedTunableNumber(name + "/Gains/kSetpoint", 0);
    io.setGains(gains);
    io.setVoltage(0);
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> {
          gainsValid =
              java.util.Arrays.stream(values).allMatch(v -> Double.isFinite(v) && v >= 0)
                  && values[6] > 0;
          if (gainsValid)
            io.setGains(
                new FlywheelGains(
                    values[0], values[1], values[2], values[3], values[4], values[5], values[6],
                    values[7]));
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kMaxAccel,
        kTolerance);
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    if (active && (getCurrentCommand() == null || getCurrentCommand() == getDefaultCommand()))
      velocitySetpoint = kSetpoint.get();
    if (!DriverStation.isTeleopEnabled()
        || !DriverStation.isJoystickConnected(0)
        || !inputs.connected
        || !gainsValid
        || !Double.isFinite(velocitySetpoint)
        || !Double.isFinite(inputs.velocity)) stop();
    if (active) io.setVelocity(Math.max(0, velocitySetpoint));
    else io.setVoltage(0);
    Logger.recordOutput(getName() + "/Target", velocitySetpoint);
    Logger.recordOutput(getName() + "/ControlMode", active ? "VELOCITY" : "VOLTAGE");
    Logger.recordOutput(getName() + "/GainsValid", gainsValid);
    Logger.recordOutput(getName() + "/isFinished", isFinished());
    Logger.recordOutput(
        getName() + "/CommandOwner",
        getCurrentCommand() == null ? "None" : getCurrentCommand().getName());
  }

  public void setVelocity(double velocity) {
    velocitySetpoint = velocity;
    active = true;
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  public boolean isFinished() {
    return active && Math.abs(inputs.velocity - velocitySetpoint) < kTolerance.get();
  }

  public static Command holdVelocity(Flywheel flywheel, DoubleSupplier velocity) {
    return flywheel.runEnd(
        () -> {
          flywheel.setVelocity(velocity.getAsDouble());
        },
        flywheel::stop);
  }

  public Command stopCommand() {
    return run(this::stop);
  }

  public void stop() {
    active = false;
    velocitySetpoint = 0;
    io.setVoltage(0);
  }
}
