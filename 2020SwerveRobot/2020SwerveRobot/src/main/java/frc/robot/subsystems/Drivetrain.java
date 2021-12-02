/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.SwerveMode;

public class Drivetrain extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public SwerveModule moduleFL, moduleFR, moduleBL, moduleBR;
    double angleFL, angleFR, angleBL, angleBR;
    SwerveMode m_mode;

    public Drivetrain() {
        m_mode = SwerveMode.CRAB; //
        moduleFL = new SwerveModule(Constants.SWERVE_FL_OFFSET_ANGLE, Constants.SWERVE_FL_ENCODER_PORT, Constants.CAN_ID_DRIVING_FL, Constants.CAN_ID_STEERING_FL, "FL");
        moduleFR = new SwerveModule(Constants.SWERVE_FR_OFFSET_ANGLE, Constants.SWERVE_FR_ENCODER_PORT, Constants.CAN_ID_DRIVING_FR, Constants.CAN_ID_STEERING_FR, "FR");
        moduleBL = new SwerveModule(Constants.SWERVE_BL_OFFSET_ANGLE, Constants.SWERVE_BL_ENCODER_PORT, Constants.CAN_ID_DRIVING_BL, Constants.CAN_ID_STEERING_BL, "BL");
        moduleBR = new SwerveModule(Constants.SWERVE_BR_OFFSET_ANGLE, Constants.SWERVE_BR_ENCODER_PORT, Constants.CAN_ID_DRIVING_BR, Constants.CAN_ID_STEERING_BR, "BR");
        resetEncoders();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateDashboard();
    }

	public void stop() {
        moduleFL.stop();
        moduleFR.stop();
        moduleBL.stop();
        moduleBR.stop();

    }
	public void resetEncoders() {
        moduleFL.resetEncoders();
        moduleFR.resetEncoders();
        moduleBL.resetEncoders();
        moduleBR.resetEncoders();
	}
	public void setAngle(double angle) {
        if (m_mode == SwerveMode.CAR)
        {
            moduleFL.setAngle(angle);
            moduleFR.setAngle(angle);
            moduleBL.setAngle(0);
            moduleBR.setAngle(0);
        }
        if (m_mode == SwerveMode.CAR_X)
        {
            moduleFL.setAngle(angle);
            moduleFR.setAngle(0);
            moduleBL.setAngle(angle);
            moduleBR.setAngle(0);
        }
        if (m_mode == SwerveMode.TANK)
        {
            moduleFL.setAngle(0);
            moduleFR.setAngle(0);
            moduleBL.setAngle(0);
            moduleBR.setAngle(0);
        }
        if (m_mode == SwerveMode.TANK_X)
        {
            moduleFL.setAngle(90);
            moduleFR.setAngle(90);
            moduleBL.setAngle(90);
            moduleBR.setAngle(90);
        }
        if (m_mode == SwerveMode.CRAB)
        {
            moduleFL.setAngle(angle);
            moduleFR.setAngle(angle);
            moduleBL.setAngle(angle);
            moduleBR.setAngle(angle);
        }
        if (m_mode == SwerveMode.CRAB_X)
        {
            moduleFL.setAngle(90+angle);
            moduleFR.setAngle(90+angle);
            moduleBL.setAngle(90+angle);
            moduleBR.setAngle(90+angle);
        }
        if (m_mode == SwerveMode.SNAKE)
        {
            moduleFL.setAngle(angle);
            moduleFR.setAngle(angle);
            moduleBL.setAngle(-angle);
            moduleBR.setAngle(-angle);
        }
        if (m_mode == SwerveMode.SNAKE_X)
        {
            moduleFL.setAngle(angle);
            moduleFR.setAngle(-angle);
            moduleBL.setAngle(angle);
            moduleBR.setAngle(-angle);
        }
	}

	public double getFrontRightEncoder() {
		return 0;
    }
    public double getFrontLeftEncoder() {
		return 0;
	}

	public double readGyro() {
		return 0;
	}

    public SwerveMode getMode()
    {
        return m_mode;
    }
    public void invertMode()
    {
        if (m_mode == SwerveMode.CAR)
            m_mode = SwerveMode.CAR_X;
        if (m_mode == SwerveMode.CAR_X)
            m_mode = SwerveMode.CAR;
        if (m_mode == SwerveMode.SNAKE)
            m_mode = SwerveMode.SNAKE_X;
        if (m_mode == SwerveMode.SNAKE_X)
            m_mode = SwerveMode.SNAKE;
        if (m_mode == SwerveMode.TANK)
            m_mode = SwerveMode.TANK_X;
        if (m_mode == SwerveMode.TANK_X)
            m_mode = SwerveMode.TANK;
        if (m_mode == SwerveMode.CRAB)
            m_mode = SwerveMode.CRAB_X;
        if (m_mode == SwerveMode.CRAB_X)
            m_mode = SwerveMode.CRAB;
    }

    public void setMode(SwerveMode mode)
    {
        m_mode = mode;
    }

    public void updateDashboard()
    {
        moduleFL.updateDashboard();
        moduleFR.updateDashboard();
        moduleBL.updateDashboard();
        moduleBR.updateDashboard();
    }

}
