/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    public static int EJECT_TIME = 2;
    public static int INTAKE_TIME = 2;
    public static int POWERCELL_STALL_CURRENT = 2;


    public static double DISTANCE_TOLERANCE = 0.08;
    public static double SLOW_DOWN_DISTANCE = 0.4;
    public static double AUTO_DRIVE_SPEED = 0.2;
    public static double AUTO_DRIVE_SLOW_SPEED = 0.15;
    public static double TURNING_THRESHOLD = 60;
    public static double ANGLE_TOLERANCE = 5;


    public static double SWERVE_FL_OFFSET_ANGLE = 0.41;
    public static double SWERVE_FR_OFFSET_ANGLE = -0.02;
    public static double SWERVE_BL_OFFSET_ANGLE = -0.1;
    public static double SWERVE_BR_OFFSET_ANGLE = 0.14;


    public static int SWERVE_FL_ENCODER_PORT = 0;
    public static int SWERVE_FR_ENCODER_PORT = 1;
    public static int SWERVE_BL_ENCODER_PORT = 2;
    public static int SWERVE_BR_ENCODER_PORT = 3;


    public static int CAN_ID_DRIVING_FL = 1;
    public static int CAN_ID_STEERING_FL = 2;

    public static int CAN_ID_DRIVING_FR = 3;
    public static int CAN_ID_STEERING_FR = 4;

    public static int CAN_ID_DRIVING_BL = 5;
    public static int CAN_ID_STEERING_BL = 6;

    public static int CAN_ID_DRIVING_BR = 7;
    public static int CAN_ID_STEERING_BR = 8;

    public static int CAN_ID_CLIMBER_WINCH = 10;
    public static int CAN_ID_CLIMBER_DEPLOY = 11;

    public static int CAN_ID_SHOOTER_A = 20;
    public static int CAN_ID_SHOOTER_B = 21;

    
    public static int CAN_ID_INTAKE = 30;
    
}