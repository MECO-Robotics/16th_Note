package frc.robot.subsystems.fuel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.fuel.FuelConstants.Settings;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** AdvantageScope-editable angles; no motor configuration or reference writes. */
final class FuelAngleTuning {
  final LoggedNetworkNumber feed;
  final LoggedNetworkNumber agitation;
  private double seenFeed;
  private double seenAgitation;
  private boolean pending;
  private final FuelAngleStore store;

  FuelAngleTuning(Settings settings) {
    this(settings, null);
  }

  FuelAngleTuning(Settings settings, FuelAngleStore store) {
    this.store = store;
    double upper = settings == null ? 0 : settings.feedDegrees();
    double lower = settings == null ? 0 : settings.agitationDegrees();
    double[] saved = store == null ? new double[0] : store.load();
    if (saved.length == 2) {
      upper = saved[0];
      lower = saved[1];
      // Real hardware may still need its full configuration. Seed the staged angles
      // without manufacturing a sequence, configuring controllers, or a reference.
      if (settings == null) {
        SmartDashboard.putNumber("Fuel/Config/FeedDegrees", upper);
        SmartDashboard.putNumber("Fuel/Config/AgitationDegrees", lower);
      }
    }
    feed = new LoggedNetworkNumber("/Tuning/Fuel/FeedDegrees", upper);
    agitation = new LoggedNetworkNumber("/Tuning/Fuel/AgitationDegrees", lower);
    feed.set(upper);
    agitation.set(lower);
    // Revalidate saved targets against the current configuration before accepting them.
    pending = true;
  }

  void synchronize(Settings settings) {
    // Remember cached reads so an old value cannot undo a full configuration apply
    // before LoggedNetworkNumber refreshes on the next robot loop.
    pending = false;
    seenFeed = feed.get();
    seenAgitation = agitation.get();
    feed.set(settings == null ? 0 : settings.feedDegrees());
    agitation.set(settings == null ? 0 : settings.agitationDegrees());
    if (settings != null) {
      try {
        validate(settings, settings.feedDegrees(), settings.agitationDegrees());
        remember(settings);
      } catch (IllegalArgumentException ignored) {
        // An incomplete full configuration must not erase the last accepted pair.
      }
    }
  }

  static Settings validate(Settings settings, double feed, double agitation) {
    Settings next = settings.withIntakeAngles(feed, agitation);
    if (feed <= next.minDegrees() || feed >= next.intakeDegrees())
      throw new IllegalArgumentException("Feed angle must be above pickup and clear of upper stop");
    return next;
  }

  void remember(Settings settings) {
    if (store != null) store.save(settings.feedDegrees(), settings.agitationDegrees());
  }

  void update(FuelSystem system) {
    double upper = feed.get();
    double lower = agitation.get();
    if (!pending
        && Double.compare(upper, seenFeed) == 0
        && Double.compare(lower, seenAgitation) == 0) return;
    // Hardware edits wait until disabled; simulation edits apply live. Invalid edits stay pending.
    pending = !system.tuneIntakeAngles(upper, lower);
    if (pending) return;
    if (store != null) store.save(upper, lower);
    seenFeed = upper;
    seenAgitation = lower;
  }
}
