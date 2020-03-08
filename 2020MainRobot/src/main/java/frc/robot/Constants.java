package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

//import frc.robot.autonomous.Coordinate;

public class Constants {
    public static boolean DEBUG_DRIVETRAIN = true;
    public static boolean DEBUG_CLIMBER = false;
    public static boolean DEBUG_SHOOTER = true;
    public static boolean DEBUG_INTAKE = false;
    public static boolean DEBUG_INJECTOR = false;
    public static boolean DEBUG_COLOUR_SENSOR = false;
    public static boolean DEBUG = true;

    public static int EJECT_TIME = 2;
    public static int INTAKE_TIME = 2;
    public static int POWERCELL_STALL_CURRENT = 2;

    public static double DISTANCE_TOLERANCE = 0.08;
    public static double SLOW_DOWN_DISTANCE = 0.4;
    public static double AUTO_DRIVE_SPEED = 0.2;
    public static double AUTO_DRIVE_SLOW_SPEED = 0.15;
    public static double TURNING_THRESHOLD = 60;
    public static double ANGLE_TOLERANCE = 0.5;



    // Auto Constants
    public static double kRamseteB = 0;
    public static double kRamseteZeta = 0;

    // Auto drivetrain
    public static double ksVolts = 1.67;
    public static double kvVoltSecondsPerMeter = 2.34;
    public static double kaVoltSecondsSquaredPerMeter = 0.401;
    public static double kPDriveVel = 0;
    public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.2);
    public static double kMaxSpeedMetersPerSecond = 2;
    public static double kMaxAccelerationMetersPerSecondSquared = 1;

    
}