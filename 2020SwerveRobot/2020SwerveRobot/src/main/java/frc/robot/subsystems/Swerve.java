/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Swerve extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    double m_offset;
    double m_angle;

    CANSparkMax driveMotor;
    TalonSRX steeringMotor;


    public Swerve(double angleOffset, int driveChannel, int steerChannel) {
        setAngleOffset(angleOffset);
        m_angle = 0;
        driveMotor = new CANSparkMax(driveChannel, MotorType.kBrushless);
        steeringMotor = new TalonSRX(steerChannel);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double getAngle(){
        //Gets Angle in radians
        return 0;
    }

    public void setAngle(double angle){
        //Sets Angle in radians

    }

    public void rotateAngle(double target_angle){
        //Rotats module to target angle
    }

    public double getSpeed(){
        //Gets speed in m/s
        return driveMotor.get();

    }

    public double getRawSpeed(){
        //Get motor power (-1 to 1)
        return driveMotor.get();
    }

    public void setSpeed(double speed){
        //Sets motor power to appropriate value
        driveMotor.set(speed);

    }

    public void setToZeroAngle(){
        //Sets current encoder value to zero
        setAngle(0);
    }

    public void stop(){
        //Stops all motors
        driveMotor.set(0);
        steeringMotor.set(ControlMode.PercentOutput, 0);

    }

    public void setAngleOffset(double angleOffset){
        //
        m_offset = angleOffset;
    }
}
