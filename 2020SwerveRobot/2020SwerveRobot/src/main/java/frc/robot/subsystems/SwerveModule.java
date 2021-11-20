/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

public class SwerveModule extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    double m_offset;
    double m_angle;
    double m_steering_pulses;
    double m_steering_sp;
    String m_pos;

    Boolean m_inverted;
    Boolean m_steering, m_driving;

    CANSparkMax m_driveMotor;
    CANSparkMax m_steeringMotor;

    CANPIDController m_steering_pidController, m_driving_pidController;
    CANEncoder m_steering_encoder;

    public double steering_kP, steering_kI, steering_kD, steering_kIz, steering_kFF, steering_kMaxOutput, steering_kMinOutput, steering_maxRPM, steering_m_setpoint;
    public double driving_kP, driving_kI, driving_kD, driving_kIz, driving_kFF, driving_kMaxOutput, driving_kMinOutput, driving_maxRPM, driving_m_setpoint;

    /** Swerve Drive drive mode */
    public enum DriveMode {
        OPEN_LOOP,
        CLOSED_LOOP,
        TELEOP,
        TRAJECTORY,
        AZIMUTH
    }

    public SwerveModule(double angleOffset, int driveChannel, int steerChannel, String pos) {
        setAngleOffset(angleOffset);
        m_angle = 0;
        m_driveMotor = new CANSparkMax(driveChannel, MotorType.kBrushless);
        m_steeringMotor = new CANSparkMax(steerChannel, MotorType.kBrushless);

        m_steering_encoder = m_steeringMotor.getAlternateEncoder();


        m_driveMotor.restoreFactoryDefaults();
        m_steeringMotor.restoreFactoryDefaults();

        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_steeringMotor.setIdleMode(IdleMode.kBrake);
        
        m_driveMotor.setSmartCurrentLimit(15);
        m_steeringMotor.setSmartCurrentLimit(15);

        m_pos = pos;

        m_inverted = false;
        m_steering = true;
        m_driving = true;

        // PID coefficients
        steering_kP = 0.1; 
        steering_kI = 0.0;
        steering_kD = 0.0; 
        //steering_kI = 0.000002;
        //steering_kD = 0.000004; 
        //kIz = 500; // Error process value must be within before I is used.
        steering_kFF = 0; 
        steering_m_setpoint = 0;
        steering_kMaxOutput = 0.7; 
        steering_kMinOutput = -0.7;
        steering_maxRPM = 4500;

        m_steering_pidController = m_steeringMotor.getPIDController();
        
        // set PID coefficients
        m_steering_pidController.setP(steering_kP);
        m_steering_pidController.setI(steering_kI);
        m_steering_pidController.setD(steering_kD);
        m_steering_pidController.setIZone(steering_kIz);
        m_steering_pidController.setFF(steering_kFF);
        m_steering_pidController.setOutputRange(steering_kMinOutput, steering_kMaxOutput);

        // PID coefficients
        driving_kP = 0.01; 
        driving_kI = 0.00000;
        driving_kD = 0.00000; 
        //driving_kI = 0.000002;
        //driving_kD = 0.000004; 
        //kIz = 500; // Error process value must be within before I is used.
        driving_kFF = 0; 
        driving_m_setpoint = 0;
        driving_kMaxOutput = 0.2; 
        driving_kMinOutput = -0.2;
        driving_maxRPM = 4500;

        m_driving_pidController = m_driveMotor.getPIDController();
        
        // set PID coefficients
        m_driving_pidController.setP(driving_kP);
        m_driving_pidController.setI(driving_kI);
        m_driving_pidController.setD(driving_kD);
        m_driving_pidController.setIZone(driving_kIz);
        m_driving_pidController.setFF(driving_kFF);
        m_driving_pidController.setOutputRange(driving_kMinOutput, driving_kMaxOutput);


        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", steering_kP);
        SmartDashboard.putNumber("I Gain", steering_kI);
        SmartDashboard.putNumber("D Gain", steering_kD);
        SmartDashboard.putNumber("I Zone", steering_kIz);
        SmartDashboard.putNumber("Feed Forward", steering_kFF);
        SmartDashboard.putNumber("Set Point", steering_m_setpoint);
        SmartDashboard.putNumber("Max Output", steering_kMaxOutput);
        SmartDashboard.putNumber("Min Output", steering_kMinOutput);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (m_steering)
        {
            // read PID coefficients from SmartDashboard
            double p = SmartDashboard.getNumber("P Gain", 0);
            double i = SmartDashboard.getNumber("I Gain", 0);
            double d = SmartDashboard.getNumber("D Gain", 0);
            double iz = SmartDashboard.getNumber("I Zone", 0);
            double ff = SmartDashboard.getNumber("Feed Forward", 0);
            double max = SmartDashboard.getNumber("Max Output", 0);
            double min = SmartDashboard.getNumber("Min Output", 0);
            m_steering_sp = SmartDashboard.getNumber("Set Point", 0);

            // if PID coefficients on SmartDashboard have changed, write new values to controller
            if((p != steering_kP)) { m_steering_pidController.setP(p); steering_kP = p; }
            if((i != steering_kI)) { m_steering_pidController.setI(i); steering_kI = i; }
            if((d != steering_kD)) { m_steering_pidController.setD(d); steering_kD = d; }
            if((iz != steering_kIz)) { m_steering_pidController.setIZone(iz); steering_kIz = iz; }
            if((ff != steering_kFF)) { m_steering_pidController.setFF(ff); steering_kFF = ff; }
            if((max != steering_kMaxOutput) || (min != steering_kMinOutput)) { 
            m_steering_pidController.setOutputRange(min, max); 
            steering_kMinOutput = min; steering_kMaxOutput = max; 
            }
            m_steering_pidController.setReference(steering_m_setpoint, ControlType.kPosition);
        }
        else
        {
            m_steeringMotor.set(0);
        }
        if (m_driving)
        {
            m_driveMotor.set(driving_m_setpoint/3);
            /*m_driving_pidController.setOutputRange(driving_kMinOutput, driving_kMaxOutput); 
            m_driving_pidController.setReference(driving_m_setpoint, ControlType.kVelocity);*/
        }
        else
        {
            m_driveMotor.set(0);
        }
    }

    public double getAngle(){
        return m_steering_encoder.getPosition();
    }

    public void resetEncoders()
    {
        m_driveMotor.getEncoder().setPosition(0);
        m_steeringMotor.getEncoder().setPosition(0);
    }
    public void setAngle(double angle){
        //Sets Angle in encoder pulses
        m_steering = true;
        steering_m_setpoint = angle;
    }

    public void rotateAngle(double target_angle){
        //Rotats module to target angle
    }

    public double getSpeed(){
        //Gets speed in m/s
        return m_driveMotor.getEncoder().getVelocity();
    }
    public double getSteerSpeed(){
        return m_steeringMotor.get();
    }

    public double getRawSpeed(){
        return m_driveMotor.get();
    }

    public void setSpeed(double speed){
        //Sets motor power to appropriate value
        if (m_inverted)
        {
            m_driveMotor.set(-speed);
        }
        else 
        {
            m_driveMotor.set(speed);
        }
        //m_steeringMotor.set(speed);
    }
    public void setVelocity(double velocity){
        if (m_inverted)
        {
            driving_m_setpoint = -velocity;
        }
        else 
        {
            driving_m_setpoint = velocity;
        }
        //m_steeringMotor.set(speed);
    }

    public void setToZeroAngle(){
        //Sets current encoder value to zero
        setAngle(0);
    }

    public void stop(){
        m_driveMotor.set(0);

        m_steering = false;
        m_steeringMotor.set(0);
    }

    public void setAngleOffset(double angleOffset){
        //
        m_offset = angleOffset;
    }
    public void updateDashboard()
    {
        SmartDashboard.putNumber(m_pos+"_DrivingSP", driving_m_setpoint);
        SmartDashboard.putNumber(m_pos+"_Speed", getSpeed());
        SmartDashboard.putNumber(m_pos+"_RawDrivespeed", getRawSpeed());
        SmartDashboard.putNumber(m_pos+"_RawSteerSpeed", getSteerSpeed());
        SmartDashboard.putNumber(m_pos+"_Angle", getAngle());
        SmartDashboard.putNumber(m_pos+"_PV", m_steeringMotor.getEncoder().getPosition());
        SmartDashboard.putNumber(m_pos+"_SP", steering_m_setpoint);
    }
}
