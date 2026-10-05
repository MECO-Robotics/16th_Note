package frc.robot.subsystems.fuel;

import static frc.robot.constants.fuel.FuelConfiguration.DEGREES_PER_MOTOR_ROTATION;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.fuel.FuelConfiguration;
import frc.robot.constants.fuel.FuelConfiguration.Coupling;
import frc.robot.constants.fuel.FuelConfiguration.Role;
import java.util.EnumMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

/** Five NEOs; every output path checks the runtime mode and verified configuration. */
public final class FuelIOSparkMax implements FuelIO {
  private final Map<Role, FuelMotor> motors = new EnumMap<>(Role.class);
  private final Map<Role, FuelMotor.Sample> samples = new EnumMap<>(Role.class);
  private final Map<Role, Boolean> configured = new EnumMap<>(Role.class);
  private final Function<Role, FuelMotor> factory;
  private final Supplier<FuelMode> mode;
  private final BooleanSupplier brownedOut;
  private final DoubleSupplier clock;
  private FuelConfiguration config;
  private ProfiledPIDController pivotController;
  private boolean pivotHolding;
  private boolean referenced;
  private boolean referencePending;
  private double referenceRequested;
  private double lastRead = Double.NEGATIVE_INFINITY;
  private String referenceStatus = "Not referenced";
  private MotorSafety safety;

  public FuelIOSparkMax(FuelConfiguration config) {
    this(
        config,
        SparkFuelMotor::new,
        FuelMode::current,
        RobotController::isBrownedOut,
        Timer::getFPGATimestamp);
    safety =
        new MotorSafety() {
          @Override
          public void stopMotor() {
            FuelIOSparkMax.this.stop();
          }

          @Override
          public String getDescription() {
            return "Fuel mechanisms";
          }
        };
    safety.setExpiration(.1);
    safety.setSafetyEnabled(false);
  }

  public FuelIOSparkMax(
      FuelConfiguration config,
      Function<Role, FuelMotor> factory,
      Supplier<FuelMode> mode,
      BooleanSupplier brownedOut,
      DoubleSupplier clock) {
    this.config = config;
    this.factory = factory;
    this.mode = mode;
    this.brownedOut = brownedOut;
    this.clock = clock;
    configure(config);
  }

  @Override
  public boolean configure(FuelConfiguration next) {
    if (mode.get() != FuelMode.DISABLED) return false;
    stop();
    invalidateReference("Configuration changed; reference again");
    config = next;
    samples.clear();
    lastRead = Double.NEGATIVE_INFINITY;
    for (Role role : Role.values()) {
      configured.put(role, false);
      if (!config.device(role).issues().isEmpty()) continue;
      if (role == Role.SHOOTER_SECONDARY
          && config.coupling() == Coupling.COUPLED_FOLLOWER
          && !config.shooterIssues().isEmpty()) continue;
      FuelMotor motor = motors.computeIfAbsent(role, factory);
      boolean follower =
          role == Role.SHOOTER_SECONDARY && config.coupling() == Coupling.COUPLED_FOLLOWER;
      boolean success =
          motor.configure(
              config.device(role),
              follower ? Role.SHOOTER_PRIMARY.canId : null,
              config.followerInverted());
      success &= motor.clearFaults();
      configured.put(role, success);
      motor.stop();
    }
    var p = config.pivot();
    pivotController =
        p.profileReady()
            ? new ProfiledPIDController(
                p.kP(),
                0,
                p.kD(),
                new TrapezoidProfile.Constraints(
                    p.maxDegreesPerSecond(), p.maxDegreesPerSecondSquared()))
            : null;
    return allConfigured();
  }

  private boolean allConfigured() {
    return java.util.Arrays.stream(Role.values()).allMatch(r -> configured.getOrDefault(r, false));
  }

  private boolean healthy(Role role) {
    var s = samples.get(role);
    return configured.getOrDefault(role, false)
        && s != null
        && s.ok()
        && Double.isFinite(s.rotations())
        && Double.isFinite(s.rpm())
        && !s.reset()
        && !s.brownout()
        && s.faults() == 0
        && clock.getAsDouble() - lastRead >= 0
        && clock.getAsDouble() - lastRead <= .1
        && !brownedOut.getAsBoolean();
  }

  private double positionDegrees() {
    return samples.get(Role.PIVOT).rotations()
        * config.device(Role.PIVOT).feedbackSign()
        * DEGREES_PER_MOTOR_ROTATION;
  }

  @Override
  public void updateInputs(FuelIOInputs inputs) {
    lastRead = clock.getAsDouble();
    for (Role role : Role.values()) {
      int i = role.ordinal();
      FuelMotor motor = motors.get(role);
      FuelMotor.Sample s =
          motor == null
              ? new FuelMotor.Sample(false, 0, 0, 0, 0, 0, false, false, 0, 0)
              : motor.read();
      samples.put(role, s);
      inputs.configured[i] = configured.getOrDefault(role, false);
      inputs.connected[i] = s.ok();
      inputs.motorRotations[i] = s.rotations();
      inputs.motorRpm[i] = s.rpm();
      inputs.currentAmps[i] = s.currentAmps();
      inputs.appliedVolts[i] = s.appliedVolts();
      inputs.temperatureCelsius[i] = s.temperatureCelsius();
      inputs.resetDetected[i] = s.reset();
      inputs.brownoutDetected[i] = s.brownout();
      inputs.controllerFaults[i] = s.faults();
      // A reset may have discarded volatile configuration. Reapply deliberately while disabled.
      if (s.reset() || s.brownout()) configured.put(role, false);
      inputs.configured[i] = configured.getOrDefault(role, false);
    }
    if (!healthy(Role.PIVOT))
      invalidateReference("Pivot disconnected, reset, browned out or unhealthy");
    if (brownedOut.getAsBoolean()) invalidateReference("Robot brownout");
    if (referencePending) {
      double referenceAge = clock.getAsDouble() - referenceRequested;
      if (mode.get() != FuelMode.DISABLED)
        invalidateReference("Enabled during reference verification");
      else if (!Double.isFinite(referenceAge) || referenceAge < 0 || referenceAge > .5)
        invalidateReference("Encoder zero verification timed out");
      else if (healthy(Role.PIVOT) && Math.abs(positionDegrees()) <= .1) {
        double max =
            config.pivot().travelKnown()
                ? config.pivot().maxDegrees() / DEGREES_PER_MOTOR_ROTATION
                : 0;
        boolean positive = config.device(Role.PIVOT).feedbackSign() == 1;
        boolean limitsOk =
            motors
                .get(Role.PIVOT)
                .softLimits(
                    positive ? 0 : -max,
                    positive ? max : 0,
                    positive || config.pivot().travelKnown(),
                    !positive || config.pivot().travelKnown());
        referenced = limitsOk;
        referencePending = false;
        referenceStatus = limitsOk ? "Upper reference verified" : "Soft-limit configuration failed";
        if (!limitsOk) configured.put(Role.PIVOT, false);
      }
    }
    for (Role role : Role.values())
      inputs.configured[role.ordinal()] = configured.getOrDefault(role, false);
    inputs.intakeAppliedDuty = samples.get(Role.ROLLERS).appliedDuty();
    inputs.indexerAppliedDuty = samples.get(Role.INDEXER).appliedDuty();
    inputs.available = allConfigured();
    inputs.healthy = java.util.Arrays.stream(Role.values()).allMatch(this::healthy);
    inputs.shooterPrimaryConnected = healthy(Role.SHOOTER_PRIMARY);
    inputs.shooterSecondaryConnected = healthy(Role.SHOOTER_SECONDARY);
    inputs.indexerConnected = healthy(Role.INDEXER);
    inputs.referenced = referenced;
    inputs.pivotDegrees = samples.containsKey(Role.PIVOT) ? positionDegrees() : 0;
    inputs.shooterPrimaryMotorRpm =
        samples.get(Role.SHOOTER_PRIMARY).rpm()
            * config.device(Role.SHOOTER_PRIMARY).feedbackSign();
    inputs.shooterSecondaryMotorRpm =
        samples.get(Role.SHOOTER_SECONDARY).rpm()
            * config.device(Role.SHOOTER_SECONDARY).feedbackSign();
    // This is a successful-read health gate, not a measurement of CAN frame age.
    inputs.ageSeconds = inputs.healthy ? 0 : .2;
    inputs.referenceStatus = referenceStatus;
    inputs.configurationStatus =
        config.deviceIssues().isEmpty()
            ? (allConfigured()
                ? "Configured"
                : "Controller configuration/reset failure; reapply disabled")
            : String.join("; ", config.deviceIssues());
  }

  @Override
  public boolean confirmUpperReference() {
    if (mode.get() != FuelMode.DISABLED || !healthy(Role.PIVOT)) return false;
    stop();
    invalidateReference("New upper reference requested");
    if (!motors.get(Role.PIVOT).zeroEncoder()) {
      referenceStatus = "Encoder zero write failed";
      return false;
    }
    referencePending = true;
    referenceRequested = clock.getAsDouble();
    referenceStatus = "Verifying encoder zero";
    return true;
  }

  @Override
  public void invalidateReference(String reason) {
    referenced = false;
    referencePending = false;
    referenceStatus = reason;
    pivotHolding = false;
  }

  @Override
  public void apply(FuelSequencer.Output output) {
    if ((mode.get() != FuelMode.TELEOP && mode.get() != FuelMode.AUTONOMOUS)
        || !config.automaticIssues().isEmpty()
        || !referenced
        || !java.util.Arrays.stream(Role.values()).allMatch(this::healthy)
        || !finite(output)) {
      stop();
      return;
    }
    if (output.pivotEnabled()) {
      double position = positionDegrees();
      if (position < -.5 || position > config.pivot().maxDegrees() + .5) {
        stop();
        invalidateReference("Pivot outside measured travel");
        return;
      }
      if (!pivotHolding) pivotController.reset(position);
      pivotHolding = true;
      double goal = MathUtil.clamp(output.pivotDegrees(), 0, config.pivot().maxDegrees());
      double voltage = pivotController.calculate(position, goal) + config.pivot().gravityVolts();
      if ((position <= 0 && voltage < 0)
          || (position >= config.pivot().maxDegrees() && voltage > 0)) voltage = 0;
      volts(Role.PIVOT, voltage);
    } else {
      pivotHolding = false;
      motors.get(Role.PIVOT).stop();
    }
    speed(
        Role.SHOOTER_PRIMARY,
        output.shooterPrimaryMotorRpm(),
        config.sequence().shooterPrimaryMotorRpm());
    if (config.coupling() != Coupling.COUPLED_FOLLOWER)
      speed(
          Role.SHOOTER_SECONDARY,
          output.shooterSecondaryMotorRpm(),
          config.sequence().shooterSecondaryMotorRpm());
    volts(Role.ROLLERS, MathUtil.clamp(output.intakeDuty(), -1, 1) * 12);
    volts(Role.INDEXER, MathUtil.clamp(output.indexerDuty(), -1, 1) * 12);
    if (safety != null) {
      safety.feed();
      safety.setSafetyEnabled(true);
    }
  }

  private static boolean finite(FuelSequencer.Output o) {
    return Double.isFinite(o.pivotDegrees())
        && Double.isFinite(o.shooterPrimaryMotorRpm())
        && Double.isFinite(o.shooterSecondaryMotorRpm())
        && Double.isFinite(o.intakeDuty())
        && Double.isFinite(o.indexerDuty());
  }

  private void speed(Role role, double rpm, double maximum) {
    if (rpm <= 0) motors.get(role).stop();
    else motors.get(role).velocity(Math.min(rpm, maximum) * config.device(role).feedbackSign());
  }

  private void volts(Role role, double volts) {
    double limit = config.device(role).maxVolts();
    motors.get(role).voltage(MathUtil.clamp(volts, -limit, limit));
  }

  @Override
  public boolean commissioningReady(FuelCommissioning.Selection selection) {
    return switch (selection) {
      case NONE -> false;
      case PIVOT -> healthy(Role.PIVOT) && referenced;
      case ROLLERS -> healthy(Role.ROLLERS);
      case INDEXER -> healthy(Role.INDEXER);
      case SHOOTERS -> healthy(Role.SHOOTER_PRIMARY)
          && healthy(Role.SHOOTER_SECONDARY)
          && config.shooterIssues().isEmpty();
    };
  }

  @Override
  public void commissioning(FuelCommissioning.Selection selection, double volts) {
    stop();
    if (mode.get() != FuelMode.TEST || !Double.isFinite(volts) || !commissioningReady(selection))
      return;
    Role role = selection.role();
    double cap = config.device(role).testVolts();
    volts = MathUtil.clamp(volts, -cap, cap);
    if (selection == FuelCommissioning.Selection.PIVOT) {
      double p = positionDegrees();
      if (config.device(role).directionConfirmed()
          && ((p <= 0 && volts < 0)
              || (config.pivot().travelKnown() && p >= config.pivot().maxDegrees() && volts > 0)))
        return;
    }
    volts(role, volts);
    if (selection == FuelCommissioning.Selection.SHOOTERS
        && config.coupling() == Coupling.INDEPENDENT) {
      double secondaryCap = config.device(Role.SHOOTER_SECONDARY).testVolts();
      volts(Role.SHOOTER_SECONDARY, MathUtil.clamp(volts, -secondaryCap, secondaryCap));
    }
    if (safety != null) {
      safety.feed();
      safety.setSafetyEnabled(true);
    }
  }

  @Override
  public void stop() {
    if (safety != null) safety.setSafetyEnabled(false);
    motors.values().forEach(FuelMotor::stop);
    pivotHolding = false;
  }
}
