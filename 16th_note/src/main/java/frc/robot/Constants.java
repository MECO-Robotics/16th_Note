package frc.robot;

//import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;
import frc.robot.vision.DriveMode;

public class Constants {
   
    //DriveTrain
    public static class DriveTrain {
        public static final double rightMotorCANID = 0;
        public static final double leftMotorCANID = 0;
    }

    public static class Arm{

    }

    public static class Manipulator{

    }

    public static class VisionPursuit {
        public static final String CAMERA_NAME = "left";
        public static final DriveMode DRIVE_MODE = DriveMode.TANK;
        public static final double MAX_FORWARD_OUTPUT = 0.35;
        public static final double MAX_TURN_OUTPUT = 0.35;
        public static final double MAX_LINEAR_METERS_PER_SECOND = 2.5;
        public static final double MAX_ANGULAR_RADIANS_PER_SECOND = 4.0;
        public static final double MANUAL_OVERRIDE_DEADBAND = 0.15;
    }

}
