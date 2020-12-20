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
    SwerveModule moduleFL, moduleFR, moduleBL, moduleBR;
    double angleFL, angleFR, angleBL, angleBR;

    public Drivetrain() {
        moduleFL = new SwerveModule(Constants.SWERVE_FL_OFFSET_ANGLE, Constants.CAN_ID_DRIVING_FL, Constants.CAN_ID_STEERING_FL);
        moduleFR = new SwerveModule(Constants.SWERVE_FR_OFFSET_ANGLE, Constants.CAN_ID_DRIVING_FR, Constants.CAN_ID_STEERING_FR);
        moduleBL = new SwerveModule(Constants.SWERVE_BL_OFFSET_ANGLE, Constants.CAN_ID_DRIVING_BL, Constants.CAN_ID_STEERING_BL);
        moduleBR = new SwerveModule(Constants.SWERVE_BR_OFFSET_ANGLE, Constants.CAN_ID_DRIVING_BR, Constants.CAN_ID_STEERING_BR);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

	public void stop() {
    }
    public void updateDashboard()
    {

    }

	public void resetEncoders() {
	}

	public double getFrontRightEncoder() {
		return 0;
    }
    public double getFrontLeftEncoder() {
		return 0;
	}

	public void drive(double d, double e) {
	}

	public double readGyro() {
		return 0;
	}

	public void arcadeDrive(double asDouble, double asDouble2, double d, boolean b) {
	}

}
