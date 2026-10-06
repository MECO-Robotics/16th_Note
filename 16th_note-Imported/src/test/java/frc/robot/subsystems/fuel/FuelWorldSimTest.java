package frc.robot.subsystems.fuel;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.*;
import frc.robot.sim.fuel.FuelWorldSim;
import org.junit.jupiter.api.Test;

class FuelWorldSimTest {
  @Test
  void coordinatedIntakeThenShotUsesActualSubsystemRequests() {
    assertTrue(edu.wpi.first.hal.HAL.initialize(500, 0));
    edu.wpi.first.wpilibj.simulation.DriverStationSim.resetData();
    edu.wpi.first.wpilibj.simulation.DriverStationSim.setDsAttached(true);
    edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(true);
    edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
    var settings = frc.robot.constants.fuel.FuelConstants.SIMULATION;
    var io = new frc.robot.sim.fuel.FuelIOSim(settings, false);
    double[] clock = {0};
    var fuel =
        new FuelSystem(
            io, frc.robot.constants.fuel.FuelConfiguration.simulation(settings), () -> clock[0]);
    var sim = new FuelWorldSim();
    for (int i = 0; i < 6; i++) sim.addFuel(FuelWorldSim.pickupMouth(FuelWorldSim.SHOOTING_START));
    try {
      for (int i = 0; i < 400; i++) {
        clock[0] += .02;
        fuel.request(true, i > 5 && i < 100, i >= 110);
        fuel.periodic();
        sim.update(
            .02,
            FuelWorldSim.SHOOTING_START,
            true,
            fuel.collecting(),
            fuel.feeding(),
            fuel.simulatedShooterRpm(),
            FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS);
        if (i == 100) assertEquals(6, sim.held());
      }
      assertEquals(6, sim.launched());
      assertEquals(6, sim.scored());
      assertEquals("", fuel.fault());
    } finally {
      fuel.stop();
      edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().unregisterSubsystem(fuel);
      edu.wpi.first.wpilibj.simulation.DriverStationSim.resetData();
      edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
    }
  }

  private void tick(FuelWorldSim sim, Pose2d pose, boolean enabled, boolean intake, boolean feed) {
    sim.update(.02, pose, enabled, intake, feed, 3000, FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS);
  }

  @Test
  void pickupRequiresEnabledIntakeAndNeverExceedsSix() {
    var sim = new FuelWorldSim();
    var pose = FuelWorldSim.SHOOTING_START;
    for (int i = 0; i < 8; i++) sim.addFuel(FuelWorldSim.pickupMouth(pose));
    tick(sim, pose, false, true, false);
    assertEquals(0, sim.held());
    tick(sim, pose, true, false, false);
    assertEquals(0, sim.held());
    tick(sim, pose, true, true, false);
    assertEquals(6, sim.held());
    assertEquals(6, sim.heldPoses(pose).length);
    tick(sim, pose, true, true, false);
    assertEquals(6, sim.held());
  }

  @Test
  void preloadsShootAndScoreOnlyWhenFeeding() {
    var sim = new FuelWorldSim();
    sim.reset(6);
    for (int i = 0; i < 50; i++) tick(sim, FuelWorldSim.SHOOTING_START, true, false, false);
    assertEquals(6, sim.held());
    assertEquals(0, sim.launched());
    for (int i = 0; i < 200; i++) tick(sim, FuelWorldSim.SHOOTING_START, true, false, true);
    assertEquals(0, sim.held());
    assertEquals(6, sim.launched());
    assertEquals(6, sim.scored());
    assertEquals(0, sim.projectilePoses().length);
  }

  @Test
  void wrongHeadingMissesAndGroundFuelCanBeCollectedAgain() {
    var sim = new FuelWorldSim();
    sim.reset(1);
    var pose = FuelWorldSim.START;
    int before = sim.groundPoses().length;
    for (int i = 0; i < 150; i++) tick(sim, pose, true, false, true);
    assertEquals(0, sim.scored());
    assertEquals(before + 1, sim.groundPoses().length);
  }

  @Test
  void disableBlocksLaunchButInFlightFuelContinuesAndResetClearsState() {
    var sim = new FuelWorldSim();
    sim.reset(6);
    tick(sim, FuelWorldSim.SHOOTING_START, true, false, true);
    for (int i = 0; i < 100; i++) tick(sim, FuelWorldSim.SHOOTING_START, false, true, true);
    assertEquals(1, sim.launched());
    assertEquals(5, sim.held());
    assertEquals(1, sim.scored());
    sim.reset(0);
    assertEquals(0, sim.held());
    assertEquals(0, sim.scored());
    assertEquals(0, sim.launched());
    assertEquals(400, sim.groundPoses().length);
  }

  @Test
  void resetStagesExactly400NonOverlappingNeutralZoneFuel() {
    var sim = new FuelWorldSim();
    for (int preloads : new int[] {0, 6}) {
      sim.reset(preloads);
      var poses = sim.groundPoses();
      assertEquals(400, poses.length);
      assertEquals(preloads, sim.held());
      for (int i = 0; i < poses.length; i++) {
        var p = poses[i];
        assertTrue(p.getX() > FuelWorldSim.HUB_X + .6 && p.getX() < FuelWorldSim.RED_HUB_X - .6);
        assertTrue(p.getY() > 0 && p.getY() < 8.07);
        for (int j = 0; j < i; j++)
          assertTrue(p.getTranslation().getDistance(poses[j].getTranslation()) >= .15 - 1e-9);
      }
    }
  }

  @Test
  void pickupUsesRobotHeadingAndSweepsBetweenUpdates() {
    var sim = new FuelWorldSim();
    var pose = new Pose2d(2, 1, Rotation2d.fromDegrees(90));
    sim.addFuel(new Translation2d(2, 1.32));
    tick(sim, pose, true, true, false);
    assertEquals(1, sim.held());
    sim.reset(0);
    sim.addFuel(new Translation2d(2.82, 1));
    tick(sim, new Pose2d(2, 1, new Rotation2d()), true, true, false);
    assertEquals(0, sim.held());
    tick(sim, new Pose2d(3, 1, new Rotation2d()), true, true, false);
    assertEquals(1, sim.held());
  }

  @Test
  void inactiveIntakeDoesNotSweepPastFuelOnReenable() {
    var sim = new FuelWorldSim();
    sim.addFuel(new Translation2d(2.82, 1));
    tick(sim, new Pose2d(2, 1, new Rotation2d()), true, true, false);
    tick(sim, new Pose2d(2.5, 1, new Rotation2d()), false, true, false);
    tick(sim, new Pose2d(3, 1, new Rotation2d()), true, true, false);
    assertEquals(0, sim.held());
  }

  @Test
  void shooterLaunchesFromRearOppositeIntakeAtEveryHeading() {
    for (double degrees : new double[] {0, 90, 180, -90}) {
      var sim = new FuelWorldSim();
      sim.reset(1);
      var robot = new Pose2d(2, 2, Rotation2d.fromDegrees(degrees));
      var forward = new Translation2d(1, 0).rotateBy(robot.getRotation());
      tick(sim, robot, true, false, true);
      var first = sim.projectilePoses()[0].getTranslation().toTranslation2d();
      var offset = first.minus(robot.getTranslation());
      assertTrue(offset.getX() * forward.getX() + offset.getY() * forward.getY() < 0);
      tick(sim, robot, true, false, false);
      var delta = sim.projectilePoses()[0].getTranslation().toTranslation2d().minus(first);
      assertTrue(delta.getX() * forward.getX() + delta.getY() * forward.getY() < 0);
      var mouth = FuelWorldSim.pickupMouth(robot).minus(robot.getTranslation());
      assertTrue(mouth.getX() * forward.getX() + mouth.getY() * forward.getY() > 0);
    }
  }

  @Test
  void fasterLaunchSettingIncreasesBallSpeedWithoutChangingDirection() {
    var slow = new FuelWorldSim();
    var fast = new FuelWorldSim();
    slow.reset(1);
    fast.reset(1);
    var robot = new Pose2d(2, 2, new Rotation2d());
    slow.update(.02, robot, true, false, true, 3000, 5.8);
    fast.update(.02, robot, true, false, true, 3000, FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS);
    double originX = 1.85;
    double slowDistance = originX - slow.projectilePoses()[0].getX();
    double fastDistance = originX - fast.projectilePoses()[0].getX();
    assertEquals(FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS / 5.8, fastDistance / slowDistance, 1e-9);
    assertTrue(fastDistance > slowDistance);
  }

  @Test
  void hoodRemains80DegreesAtDifferentSpeeds() {
    for (double speed : new double[] {5.8, 7.5, 10}) {
      var sim = new FuelWorldSim();
      sim.reset(1);
      var robot = new Pose2d(2, 2, new Rotation2d());
      sim.update(.02, robot, true, false, true, 3000, speed, 10);
      var shot = sim.projectilePoses()[0];
      double horizontal = 1.85 - shot.getX();
      double vertical = shot.getZ() - .65 + .5 * 9.81 * .02 * .02;
      assertEquals(80, Math.toDegrees(Math.atan2(vertical, horizontal)), 1e-9);
    }
  }

  @Test
  void invalidShotParametersCannotConsumeFuel() {
    var sim = new FuelWorldSim();
    sim.reset(6);
    sim.update(.02, FuelWorldSim.SHOOTING_START, true, false, true, 3000, Double.NaN);
    sim.update(.02, FuelWorldSim.SHOOTING_START, true, false, true, 0, 7.5);
    sim.update(.02, FuelWorldSim.SHOOTING_START, true, false, true, 3000, 16);
    assertEquals(6, sim.held());
    assertEquals(0, sim.launched());
  }

  @Test
  void nineFootLimitIncludesBallRadiusAndBlocksHigherSpeedWithoutFakingTrajectory() {
    var sim = new FuelWorldSim();
    sim.reset(6);
    var awayFromHub = new Pose2d(8, 2, new Rotation2d());
    sim.update(.02, awayFromHub, true, false, true, 3000, 7.5);
    assertEquals(6, sim.held());
    assertTrue(sim.shotBlockedReason().contains("exceeds"));
    sim.update(.02, awayFromHub, true, false, true, 6000, FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS);
    assertEquals(6, sim.held());
    sim.update(.02, awayFromHub, true, false, true, 3000, FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS);
    assertEquals(5, sim.held());
    assertEquals("", sim.shotBlockedReason());
    double observedPeak = 0;
    for (int i = 0; i < 100; i++) {
      sim.update(.02, awayFromHub, true, false, false, 3000, FuelWorldSim.DEFAULT_LAUNCH_SPEED_MPS);
      for (var shot : sim.projectilePoses()) {
        observedPeak = Math.max(observedPeak, shot.getZ() + FuelWorldSim.RADIUS);
      }
    }
    assertEquals(9 * .3048, observedPeak, .002);
    assertTrue(observedPeak <= 9 * .3048 + 1e-9);
  }

  @Test
  void peakLimitIsAdjustableAndInvalidLimitsBlockNewShots() {
    var sim = new FuelWorldSim();
    sim.reset(6);
    for (double limit : new double[] {Double.NaN, Double.POSITIVE_INFINITY, 0, .7}) {
      sim.update(.02, FuelWorldSim.SHOOTING_START, true, false, true, 3000, 7.5, limit);
      assertEquals(6, sim.held());
    }
    sim.update(.02, FuelWorldSim.SHOOTING_START, true, false, true, 3000, 7.5, 12 * .3048);
    assertEquals(5, sim.held());
    assertEquals("", sim.shotBlockedReason());
  }
}
