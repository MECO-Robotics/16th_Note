package frc.robot.subsystems.fuel;

import frc.robot.constants.fuel.FuelConfiguration.Role;

/** Deadman, selection interlock and bounded bursts, independent of dashboard/network timing. */
public final class FuelCommissioning {
  public enum Selection {
    NONE,
    PIVOT,
    INDEXER_PAIR,
    SHOOTERS;

    public Role role() {
      return switch (this) {
        case PIVOT -> Role.PIVOT;
        case INDEXER_PAIR -> Role.INDEXER_PRIMARY;
        case SHOOTERS -> Role.SHOOTER_PRIMARY;
        case NONE -> throw new IllegalStateException("No mechanism selected");
      };
    }
  }

  private Selection previous = Selection.NONE;
  private boolean armed;
  private double started = Double.NaN;
  private double activeVolts;
  private String status = "Release bumper to arm";

  public String status() {
    return status;
  }

  public void reset() {
    previous = Selection.NONE;
    armed = false;
    started = Double.NaN;
  }

  public double update(
      double now,
      boolean allowed,
      boolean connected,
      boolean held,
      Selection selected,
      double requestedVolts,
      boolean ready) {
    if (!allowed || !connected || !Double.isFinite(now)) {
      reset();
      status = "Test mode, fresh request and controller connection required";
      return 0;
    }
    if (selected != previous) {
      previous = selected;
      armed = false;
      started = Double.NaN;
    }
    if (!held) {
      armed = true;
      started = Double.NaN;
      status = "Ready for held bumper";
      return 0;
    }
    if (!armed) {
      status = "Release bumper to arm";
      return 0;
    }
    if (!ready || !Double.isFinite(requestedVolts) || requestedVolts == 0) {
      armed = false;
      status = "Selected device not ready or test voltage unset";
      return 0;
    }
    if (Double.isNaN(started)) {
      started = now;
      activeVolts = requestedVolts;
    }
    if (requestedVolts != activeVolts) {
      armed = false;
      status = "Output changed; release bumper";
      return 0;
    }
    double duration = selected == Selection.PIVOT ? .25 : selected == Selection.SHOOTERS ? 5 : 1;
    if (now - started >= duration) {
      armed = false;
      status = "Burst ended; release bumper";
      return 0;
    }
    status = "Running " + selected;
    return activeVolts;
  }
}
