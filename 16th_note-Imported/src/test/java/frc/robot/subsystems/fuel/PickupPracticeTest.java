package frc.robot.subsystems.fuel;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.fuel.FuelConfiguration;
import frc.robot.constants.fuel.FuelConstants;
import frc.robot.controls.DriverControls;
import frc.robot.sim.fuel.FuelIOSim;
import frc.robot.sim.fuel.FuelPracticeSimulation;
import frc.robot.sim.tank.TankDriveIOSim;
import frc.robot.subsystems.tank.TankDrive;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class PickupPracticeTest {
  @ParameterizedTest
  @ValueSource(booleans = {true, false})
  void disabledPreparationThenBumperAndDrivingCollectSix(boolean xbox) {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    var scheduler = CommandScheduler.getInstance();
    double[] now = {0};
    var io = new FuelIOSim(FuelConstants.SIMULATION, false);
    io.invalidateReference("Startup");
    var fuel =
        new FuelSystem(io, FuelConfiguration.simulation(FuelConstants.SIMULATION), () -> now[0]);
    var drive = new TankDrive(new TankDriveIOSim());
    var practice = new FuelPracticeSimulation(drive, fuel);
    var controls = new DriverControls(0, xbox);
    var joystick = new GenericHIDSim(0);
    joystick.setAxisCount(6);
    joystick.setButtonCount(14);
    joystick.notifyNewData();
    fuel.setDefaultCommand(fuel.teleopCommand(controls));
    Runnable tick =
        () -> {
          now[0] += .02;
          scheduler.run();
          practice.periodic();
        };
    try {
      tick.run();
      assertTrue(fuel.automaticBlockReason().contains("referenced"));
      var prepare = (Command) SmartDashboard.getData("Fuel/Sim/Prepare pickup practice (disabled)");
      scheduler.schedule(prepare);
      tick.run();
      tick.run();
      assertEquals("", fuel.automaticBlockReason());
      assertEquals(400, SmartDashboard.getNumber("Fuel/Sim/GroundCount", -1));
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      for (int i = 0; i < 5; i++) tick.run();
      joystick.setRawButton(5, true);
      joystick.notifyNewData();
      for (int i = 0; i < 150; i++) {
        if (i > 50) drive.setVoltage(3, 3);
        tick.run();
      }
      assertTrue(fuel.collecting());
      assertEquals(6, SmartDashboard.getNumber("Fuel/Sim/HeldCount", -1));
      assertEquals(394, SmartDashboard.getNumber("Fuel/Sim/GroundCount", -1));
      // An enabled preparation request must not refill or reference/reset a moving simulation.
      scheduler.schedule(prepare);
      tick.run();
      assertEquals(394, SmartDashboard.getNumber("Fuel/Sim/GroundCount", -1));
      assertTrue(SmartDashboard.getString("Fuel/Sim/ResetStatus", "").contains("Rejected"));
      joystick.setRawButton(5, false);
      joystick.notifyNewData();
      for (int i = 0; i < 3; i++) tick.run();
      assertFalse(fuel.collecting());
      io.invalidateReference("Lost reference");
      tick.run();
      assertFalse(fuel.collecting());
      assertEquals(6, SmartDashboard.getNumber("Fuel/Sim/HeldCount", -1));
    } finally {
      scheduler.cancelAll();
      scheduler.unregisterSubsystem(fuel, drive);
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    }
  }
}
