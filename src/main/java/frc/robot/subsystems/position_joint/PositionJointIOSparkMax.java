package frc.robot.subsystems.position_joint;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;

/** Uses the rebuilt controller-side PID and cosine feedforward. */
public final class PositionJointIOSparkMax implements PositionJointIO {
  private final SparkMax motor;
  private final PositionJointHardwareConfig hardware;
  private final String name;
  private final double maxVolts;
  private boolean configured;
  private boolean referenced;
  private boolean connected;
  private String status = "Not configured";
  private String controlStatus = "Not commanded";
  private String controllerConfiguration = "Not read";
  private double alternateMaxVelo = Double.NaN;
  private double alternateMaxAccel = Double.NaN;

  private boolean startupReferencePending = true;

  public PositionJointIOSparkMax(
      String name, PositionJointHardwareConfig hardware, double maxVolts) {
    this.name = name;
    this.hardware = hardware;
    this.maxVolts = maxVolts;
    motor = new SparkMax(hardware.canIds()[0], MotorType.kBrushless);
    motor.stopMotor();
  }

  @Override
  public void updateInputs(PositionJointIOInputs inputs) {
    inputs.outputPosition = motor.getEncoder().getPosition();
    boolean positionOk = motor.getLastError() == REVLibError.kOk;
    inputs.velocity = motor.getEncoder().getVelocity();
    boolean velocityOk = motor.getLastError() == REVLibError.kOk;
    double bus = motor.getBusVoltage();
    boolean busOk = motor.getLastError() == REVLibError.kOk;
    inputs.appliedVolts = motor.getAppliedOutput() * bus;
    inputs.currentAmps = motor.getOutputCurrent();
    var warnings = motor.getStickyWarnings();
    boolean warningsOk = motor.getLastError() == REVLibError.kOk;
    connected = configured && positionOk && velocityOk && busOk && warningsOk && bus > 1;
    if (!connected || warnings.hasReset || warnings.brownout || RobotController.isBrownedOut()) {
      referenced = false;
      motor.stopMotor();
    }
    if (warnings.hasReset || warnings.brownout) motor.clearFaults();
    inputs.connected = connected;
    inputs.referenced = referenced;
    inputs.configurationStatus = status;
    inputs.controllerConfiguration = controllerConfiguration;
    var controller = motor.getClosedLoopController();
    inputs.controlStatus = controlStatus;
    inputs.controllerMode = controller.getControlType().name();
    inputs.controllerSetpoint = controller.getSetpoint();
    inputs.profilePosition = controller.getMAXMotionSetpointPosition();
    inputs.profileVelocity = controller.getMAXMotionSetpointVelocity();
    inputs.forwardLimitPressed = motor.getForwardLimitSwitch().isPressed();
    inputs.reverseLimitPressed = motor.getReverseLimitSwitch().isPressed();
    inputs.forwardSoftLimitReached = motor.getForwardSoftLimit().isReached();
    inputs.reverseSoftLimitReached = motor.getReverseSoftLimit().isReached();
    inputs.faultBits = motor.getFaults().rawBits;
  }

  @Override
  public void setGains(PositionJointGains gains) {
    if (!Double.isFinite(alternateMaxVelo)) alternateMaxVelo = gains.kMaxVelo();
    if (!Double.isFinite(alternateMaxAccel)) alternateMaxAccel = gains.kMaxAccel();
    var config = new SparkMaxConfig();
    config
        .inverted(hardware.reversed()[0])
        .disableFollowerMode()
        .smartCurrentLimit(hardware.currentLimit())
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12);
    config
        .encoder
        .positionConversionFactor(1.0 / hardware.gearRatio())
        .velocityConversionFactor(1.0 / (60.0 * hardware.gearRatio()));
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(gains.kP(), gains.kI(), gains.kD())
        .pid(gains.kP(), gains.kI(), gains.kD(), ClosedLoopSlot.kSlot1)
        .outputRange(-maxVolts / 12, maxVolts / 12)
        .outputRange(-maxVolts / 12, maxVolts / 12, ClosedLoopSlot.kSlot1);
    config.closedLoop.feedForward.apply(
        new FeedForwardConfig()
            .kS(gains.kS())
            .kCos(gains.kG())
            .kCosRatio(1.0)
            .kV(gains.kV())
            .kA(gains.kA())
            .kS(gains.kS(), ClosedLoopSlot.kSlot1)
            .kCos(gains.kG(), ClosedLoopSlot.kSlot1)
            .kCosRatio(1.0, ClosedLoopSlot.kSlot1)
            .kV(gains.kV(), ClosedLoopSlot.kSlot1)
            .kA(gains.kA(), ClosedLoopSlot.kSlot1));
    config
        .closedLoop
        .maxMotion
        .cruiseVelocity(gains.kMaxVelo())
        .maxAcceleration(gains.kMaxAccel())
        // Allow the profile to advance during initial tuning instead of regenerating
        // immediately while the mechanism is still overcoming static friction.
        .allowedProfileError(gains.kMaxPosition() - gains.kMinPosition())
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .cruiseVelocity(alternateMaxVelo, ClosedLoopSlot.kSlot1)
        .maxAcceleration(alternateMaxAccel, ClosedLoopSlot.kSlot1)
        .allowedProfileError(gains.kMaxPosition() - gains.kMinPosition(), ClosedLoopSlot.kSlot1)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot1);
    config
        .signals
        .maxMotionSetpointPositionAlwaysOn(true)
        .maxMotionSetpointPositionPeriodMs(20)
        .maxMotionSetpointVelocityAlwaysOn(true)
        .maxMotionSetpointVelocityPeriodMs(20)
        .setSetpointAlwaysOn(true)
        .setpointPeriodMs(20);
    config
        .softLimit
        .reverseSoftLimit(gains.kMinPosition())
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(gains.kMaxPosition())
        .forwardSoftLimitEnabled(true);
    configured = false;
    for (int attempt = 0; attempt < 5 && !configured; attempt++) {
      var result =
          motor.configure(
              config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      status = result.name();
      configured = result == REVLibError.kOk;
    }
    if (!configured) motor.stopMotor();
    if (configured) {
      var actual = motor.configAccessor.closedLoop;
      controllerConfiguration =
          "firmware="
              + motor.getFirmwareString()
              + "; P="
              + actual.getP()
              + "; output=["
              + actual.getMinOutput()
              + ","
              + actual.getMaxOutput()
              + "]"
              + "; feedback="
              + actual.getFeedbackSensor()
              + "; wrapping="
              + actual.getPositionWrappingEnabled()
              + "; allowedError="
              + actual.getAllowedClosedLoopError(com.revrobotics.spark.ClosedLoopSlot.kSlot0)
              + "; cruise="
              + actual.maxMotion.getCruiseVelocity()
              + "; acceleration="
              + actual.maxMotion.getMaxAcceleration()
              + "; profileError="
              + actual.maxMotion.getAllowedProfileError()
              + "; profileMode="
              + actual.maxMotion.getPositionMode();
    }
    if (configured && startupReferencePending) {
      // Initial reference is the configured physical home, assumed at program startup only.
      resetPosition();
      startupReferencePending = false;
    }
  }

  @Override
  public void setAlternateMotionConstraints(double maxVelo, double maxAccel) {
    alternateMaxVelo = maxVelo;
    alternateMaxAccel = maxAccel;
    var config = new SparkMaxConfig();
    config
        .closedLoop
        .maxMotion
        .cruiseVelocity(maxVelo, ClosedLoopSlot.kSlot1)
        .maxAcceleration(maxAccel, ClosedLoopSlot.kSlot1);
    configured = false;
    for (int attempt = 0; attempt < 5 && !configured; attempt++) {
      var result =
          motor.configure(
              config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      status = result.name();
      configured = result == REVLibError.kOk;
    }
    if (!configured) motor.stopMotor();
  }

  @Override
  public void setPosition(double position, double velocity) {
    setPosition(position, velocity, false);
  }

  @Override
  public void setPosition(double position, double velocity, boolean useAlternateMotionConstraints) {
    if (!configured || !connected || !Double.isFinite(position)) {
      motor.stopMotor();
      controlStatus = "Blocked: configuration, connection, or nonfinite position";
      return;
    }
    var result =
        motor
            .getClosedLoopController()
            .setSetpoint(
                position,
                ControlType.kMAXMotionPositionControl,
                useAlternateMotionConstraints ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0);
    controlStatus =
        "MAXMotion position slot " + (useAlternateMotionConstraints ? 1 : 0) + ": " + result.name();
    if (result != REVLibError.kOk) motor.stopMotor();
  }

  @Override
  public void setVoltage(double volts) {
    // This joint only uses neutral open-loop output.
    motor.stopMotor();
    controlStatus = "Neutral";
  }

  @Override
  public void resetPosition() {
    if (!DriverStation.isDisabled()) return;
    motor.stopMotor();
    motor.clearFaults();
    referenced =
        configured
            && motor.getEncoder().setPosition(hardware.encoderOffset().getRotations())
                == REVLibError.kOk;
  }

  @Override
  public String getName() {
    return name;
  }
}
