// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.vision.VisionDriveAdapter;
import frc.robot.vision.VisionDriveFactory;
import org.mecorobotics.gamepiecevision.GamePieceVisionClient;
import org.mecorobotics.gamepiecevision.GamePieceVisionClient.DriveRequest;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_robotDrive;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private GamePieceVisionClient m_gamePieceVision;
  private VisionDriveAdapter m_visionDriveAdapter;

  private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);

  @Override
  public void robotInit() {
    m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
    m_gamePieceVision =
        new GamePieceVisionClient(Constants.VisionPursuit.CAMERA_NAME);
    m_visionDriveAdapter =
        VisionDriveFactory.create(
            Constants.VisionPursuit.DRIVE_MODE,
            m_robotDrive,
            Constants.VisionPursuit.MAX_FORWARD_OUTPUT,
            Constants.VisionPursuit.MAX_TURN_OUTPUT,
            this::driveVisionSwerve,
            Constants.VisionPursuit.MAX_LINEAR_METERS_PER_SECOND,
            Constants.VisionPursuit.MAX_ANGULAR_RADIANS_PER_SECOND);
  }

  @Override
  public void disabledInit() {
    m_robotDrive.stopMotor();
    m_visionDriveAdapter.stop();
  }

  @Override
  public void teleopInit() {
    m_robotDrive.stopMotor();
    m_visionDriveAdapter.stop();
  }

  @Override
  public void teleopPeriodic() {
    double leftSpeed = -m_leftStick.getY();
    double rightSpeed = -m_rightStick.getY();
    boolean assistHeld = isAssistHeld();
    boolean manualInput = isManualOverrideInput(leftSpeed, rightSpeed);

    DriveRequest request =
        m_gamePieceVision.getRequest(assistHeld, manualInput);
    if (request.active()) {
      m_visionDriveAdapter.apply(request);
    } else {
      m_robotDrive.tankDrive(leftSpeed, rightSpeed);
    }
  }

  private boolean isAssistHeld() {
    return m_rightStick.getTrigger();
  }

  private boolean isManualOverrideInput(double leftSpeed, double rightSpeed) {
    return Math.abs(leftSpeed) > Constants.VisionPursuit.MANUAL_OVERRIDE_DEADBAND
        || Math.abs(rightSpeed) > Constants.VisionPursuit.MANUAL_OVERRIDE_DEADBAND;
  }

  private void driveVisionSwerve(double forward, double strafe, double turn) {
    // Intentionally no-op for this tank-only bot.
    // Remy (swerve bot) should implement this method by mapping to driveRobotRelative().
    // If this bot is switched to SWERVE, this method should call the drivetrain swerve API.
  }
}
