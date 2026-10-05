package frc.robot.subsystems.fuel;

import frc.robot.constants.fuel.FuelConstants.Settings;

/**
 * Generic sequence coordinates retained to verify the algorithm supports either travel direction.
 */
final class FuelTestSettings {
  static final Settings EXAMPLE =
      new Settings(0, 80, 0, 70, 50, 2, 3000, 150, .2, 4, 3, .6, .15, 3, false, .1, .5, .35);

  private FuelTestSettings() {}
}
