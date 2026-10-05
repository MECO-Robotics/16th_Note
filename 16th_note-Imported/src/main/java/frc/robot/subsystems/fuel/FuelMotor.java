package frc.robot.subsystems.fuel;

import frc.robot.constants.fuel.FuelConfiguration.Device;

/** Small motor boundary for testing configuration, reset and encoder-write failures without CAN. */
public interface FuelMotor {
  record Sample(
      boolean ok,
      double rotations,
      double rpm,
      double currentAmps,
      double appliedVolts,
      double temperatureCelsius,
      boolean reset,
      boolean brownout,
      int faults,
      double appliedDuty) {}

  boolean configure(Device device, Integer leaderId, boolean followerInverted);

  Sample read();

  boolean clearFaults();

  boolean zeroEncoder();

  boolean softLimits(
      double minRotations, double maxRotations, boolean minEnabled, boolean maxEnabled);

  void voltage(double volts);

  void velocity(double motorRpm);

  void stop();
}
