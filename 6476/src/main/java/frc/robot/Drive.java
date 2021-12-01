package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * Add your docs here.
 * 
 * @param <WPI_VictorSRX>
 */
public class Drive extends SubsystemBase {

    VictorSPX leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;
    
    public Drive() {
    
    // instantiate new motor controller objects 
    leftFrontMotor = new VictorSPX(RobotMap.leftFrontMotor);
    leftRearMotor = new VictorSPX(RobotMap.leftRearMotor);
    rightFrontMotor = new VictorSPX(RobotMap.rightFrontMotor);
    rightRearMotor = new VictorSPX(RobotMap.rightRearMotor);
     
    leftRearMotor.follow(leftFrontMotor);
    rightRearMotor.follow(rightFrontMotor);
    }
     // add manualDrive() method (which I'll keep as teleopdrive)
     public void teleopDrive(double move, double turn){
       double left =  (turn - move);
       double right = (turn + move);
       leftFrontMotor.set(ControlMode.PercentOutput, left);
       rightRearMotor.set(ControlMode.PercentOutput, right);
     }
     
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }
      


}
