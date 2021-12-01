// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColourWheel extends SubsystemBase {
  Talon colourWheelMotor;
  Servo colourWheelServo;
  /** Creates a new ColourWheel. */
  public ColourWheel() {
    colourWheelMotor = new Talon(RobotMap.colourWheel_PWM);
    colourWheelServo = new Servo(RobotMap.colourWheelServo_PWM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setServo(double position)
  {
    colourWheelServo.set(position);
  }
  public void forward()
  {
    colourWheelMotor.set(0.5);
  }
  public void reverse()
  {
    colourWheelMotor.set(-0.5);
  }
  
  public void stop()
  {
    colourWheelMotor.set(0);
  }
}
