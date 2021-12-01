// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Talon;

public class Intake {
  
  Talon intakeMotor;

  public Intake() {
    intakeMotor = new Talon(RobotMap.intake_PWM);
  }

  public void pickUp()
  {
    intakeMotor.set(0.5);
  }
  public void reverse()
  {
    intakeMotor.set(-0.5);
  }
  
  public void stop()
  {
    intakeMotor.set(0);
  }
}
