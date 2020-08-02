/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    Swerve moduleFL, moduleFR, moduleBL, moduleBR;
    double angleFL, angleFR, angleBL, angleBR;

    public Drivetrain() {
        moduleFL = new Swerve(Constants.SWERVE_FL_OFFSET_ANGLE, Constants.CAN_ID_DRIVING_FL, Constants.CAN_ID_STEERING_FL);
        moduleFR = new Swerve(Constants.SWERVE_FR_OFFSET_ANGLE, Constants.CAN_ID_DRIVING_FR, Constants.CAN_ID_STEERING_FR);
        moduleBL = new Swerve(Constants.SWERVE_BL_OFFSET_ANGLE, Constants.CAN_ID_DRIVING_BL, Constants.CAN_ID_STEERING_BL);
        moduleBR = new Swerve(Constants.SWERVE_BR_OFFSET_ANGLE, Constants.CAN_ID_DRIVING_BR, Constants.CAN_ID_STEERING_BR);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
