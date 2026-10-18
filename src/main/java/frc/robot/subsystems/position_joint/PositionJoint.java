package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Rebuilt PositionJoint tuning/IO pattern, with only this robot's required controls. */
public final class PositionJoint extends SubsystemBase {
  private final PositionJointIO io;
  private final PositionJointIOInputsAutoLogged inputs = new PositionJointIOInputsAutoLogged();
  private final LoggedTunableNumber kP, kI, kD, kS, kG, kV, kA;
  private final LoggedTunableNumber kMaxVelo,
      kMaxAccel,
      kMinPosition,
      kMaxPosition,
      kTolerance,
      kSetpoint;
  private PositionJointGains gains;
  private double goalPosition;
  private boolean active;
  private boolean gainsValid = true;
  private final double minimumBound, maximumBound;

  public PositionJoint(PositionJointIO io, PositionJointGains gains) {
    super(io.getName());
    String name = io.getName();
    this.io = io;
    this.gains = gains;
    minimumBound = gains.kMinPosition();
    maximumBound = gains.kMaxPosition();
    kP = new LoggedTunableNumber(name + "/Gains/kP", gains.kP());
    kI = new LoggedTunableNumber(name + "/Gains/kI", gains.kI());
    kD = new LoggedTunableNumber(name + "/Gains/kD", gains.kD());
    kS = new LoggedTunableNumber(name + "/Gains/kS", gains.kS());
    kG = new LoggedTunableNumber(name + "/Gains/kG", gains.kG());
    kV = new LoggedTunableNumber(name + "/Gains/kV", gains.kV());
    kA = new LoggedTunableNumber(name + "/Gains/kA", gains.kA());
    kMaxVelo = new LoggedTunableNumber(name + "/Gains/kMaxVelo", gains.kMaxVelo());
    kMaxAccel = new LoggedTunableNumber(name + "/Gains/kMaxAccel", gains.kMaxAccel());
    kMinPosition = new LoggedTunableNumber(name + "/Gains/kMinPosition", gains.kMinPosition());
    kMaxPosition = new LoggedTunableNumber(name + "/Gains/kMaxPosition", gains.kMaxPosition());
    kTolerance = new LoggedTunableNumber(name + "/Gains/kTolerance", gains.kTolerance());
    kSetpoint = new LoggedTunableNumber(name + "/Gains/kSetpoint", gains.kDefaultSetpoint());
    goalPosition = gains.kDefaultSetpoint();
    io.setGains(gains);
    SmartDashboard.putData(
        name + "/Reference home (disabled)",
        runOnce(
                () -> {
                  if (DriverStation.isDisabled()) {
                    stop();
                    io.resetPosition();
                  }
                })
            .ignoringDisable(true));
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> {
          gainsValid =
              java.util.Arrays.stream(values).allMatch(Double::isFinite)
                  && values[0] >= 0
                  && values[1] >= 0
                  && values[2] >= 0
                  && values[7] > 0
                  && values[8] > 0
                  && values[11] > 0
                  && values[9] >= minimumBound
                  && values[10] <= maximumBound
                  && values[9] < values[10];
          if (gainsValid) {
            gains =
                new PositionJointGains(
                    values[0],
                    values[1],
                    values[2],
                    values[3],
                    values[4],
                    values[5],
                    values[6],
                    values[7],
                    values[8],
                    values[9],
                    values[10],
                    values[11],
                    values[12]);
            io.setGains(gains);
          }
        },
        kP,
        kI,
        kD,
        kS,
        kG,
        kV,
        kA,
        kMaxVelo,
        kMaxAccel,
        kMinPosition,
        kMaxPosition,
        kTolerance,
        kSetpoint);
    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        () -> {
          if (getCurrentCommand() == null && active) setPosition(kSetpoint.get());
        },
        kSetpoint);
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    boolean allowed =
        DriverStation.isTeleopEnabled()
            && DriverStation.isJoystickConnected(0)
            && inputs.connected
            && gainsValid
            && Double.isFinite(inputs.outputPosition)
            && Double.isFinite(goalPosition);
    if (!allowed) stop();
    if (active && allowed)
      io.setPosition(MathUtil.clamp(goalPosition, gains.kMinPosition(), gains.kMaxPosition()), 0);
    else io.setVoltage(0);
    Logger.recordOutput(getName() + "/GoalPosition", goalPosition);
    Logger.recordOutput(getName() + "/Referenced", inputs.referenced);
    Logger.recordOutput(getName() + "/GainsValid", gainsValid);
    Logger.recordOutput(getName() + "/isFinished", isFinished());
    Logger.recordOutput(
        getName() + "/CommandOwner",
        getCurrentCommand() == null ? "None" : getCurrentCommand().getName());
    Logger.recordOutput(getName() + "/ControlMode", active ? "POSITION" : "VOLTAGE");
    Logger.recordOutput(
        getName() + "/GravityVolts",
        active ? gains.kG() * Math.cos(2 * Math.PI * inputs.outputPosition) : 0);
  }

  public void setPosition(double position) {
    goalPosition = position;
    active = true;
  }

  public double getPosition() {
    return inputs.outputPosition;
  }

  public boolean isFinished() {
    return active
        && Double.isFinite(goalPosition)
        && Math.abs(
                inputs.outputPosition
                    - MathUtil.clamp(goalPosition, gains.kMinPosition(), gains.kMaxPosition()))
            < kTolerance.get();
  }

  public static Command holdPosition(PositionJoint joint, DoubleSupplier rotations) {
    return joint.runEnd(() -> joint.setPosition(rotations.getAsDouble()), joint::stop);
  }

  public void stop() {
    active = false;
    io.setVoltage(0);
  }
}
