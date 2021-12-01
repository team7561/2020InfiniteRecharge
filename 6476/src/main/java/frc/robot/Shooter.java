// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Talon;

public class Shooter {
  
  Talon shooterMotorA, shooterMotorB;

  public Shooter() {
    shooterMotorA = new Talon(RobotMap.shooter_A_PWM);
    shooterMotorB = new Talon(RobotMap.shooter_B_PWM);

    
  }

  public void shoot(double speed)
  {
    shooterMotorA.set(-speed);
    shooterMotorB.set(speed);
  }
  
  public void stop()
  {
    shooterMotorA.set(0);
    shooterMotorB.set(0);
  }
}
