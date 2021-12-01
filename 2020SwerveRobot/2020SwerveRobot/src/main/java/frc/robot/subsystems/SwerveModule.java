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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.controller.PIDController;

public class SwerveModule extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    double m_offset;
    double m_angle;
    double m_steering_target;
    double m_steering_sp;
    double currentAngle;

    String m_pos;

    Boolean m_inverted;
    Boolean m_steering, m_driving;

    CANSparkMax m_driveMotor;
    CANSparkMax m_steeringMotor;

    CANPIDController m_driving_pidController;
    CANEncoder m_steering_encoder;

    DigitalSource absolute_encoder_source;
    DutyCycle absolute_encoder;

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

    public SwerveModule(double angleOffset, int encoderPort, int driveChannel, int steerChannel, String pos) {
        setAngleOffset(angleOffset);
        m_angle = 0;
        m_driveMotor = new CANSparkMax(driveChannel, MotorType.kBrushless);
        m_steeringMotor = new CANSparkMax(steerChannel, MotorType.kBrushless);

        absolute_encoder_source = new DigitalInput(encoderPort);
        absolute_encoder = new DutyCycle(absolute_encoder_source);

        //m_steering_encoder = m_steeringMotor.getAlternateEncoder();


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

        currentAngle = 0;
        

        // PID coefficients
        driving_kP = 0.01; 
        driving_kI = 0.00000;
        driving_kD = 0.00000; 
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
        SmartDashboard.putNumber(m_pos+"Encoder", absolute_encoder.getOutput());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        //m_steering_target = 170/360;
        currentAngle = SmartDashboard.getNumber(m_pos+"_Angle", 0)-m_offset;
        if (currentAngle < 0)
        {
            currentAngle += 1;
        }
        double error = m_steering_target-currentAngle;
        System.out.println(m_pos+"_currentAngle: " + currentAngle);
        System.out.println(m_pos+"_Target Angle: " + m_steering_target);
        System.out.println(m_pos+"_Error: " + error);
        if (m_steering)
        {
            m_steeringMotor.set(2*error);
            /*
            if (currentAngle > m_steering_target)
            {
                System.out.println("Need to go forwards");
                System.out.println(currentAngle);
                m_steeringMotor.set(0.1);
            }
            else
            {
                System.out.println("Need to go backwards");
                System.out.println(currentAngle);
                m_steeringMotor.set(-0.1);
            }
            */
            //m_steering_sp = SmartDashboard.getNumber("Set Point", 0);
            
        }
        else
        {
            m_steeringMotor.set(0);
        }
        if (m_driving)
        {
            m_driveMotor.set(driving_m_setpoint*0.99);
            /*m_driving_pidController.setOutputRange(driving_kMinOutput, driving_kMaxOutput); 
            m_driving_pidController.setReference(driving_m_setpoint, ControlType.kVelocity);*/
        }
        else
        {
            m_driveMotor.set(0);
        }
    }
    public double getPulses()
    {
        return absolute_encoder.getOutput();
    }
    public double getAngle()
    {
        return absolute_encoder.getOutput();
    }

    public void resetEncoders()
    {
        m_driveMotor.getEncoder().setPosition(0);
        m_steeringMotor.getEncoder().setPosition(0);
    }
    public void setAngle(double angle){
        m_steering_target = angle;
    }

    public void rotateAngle(double target_angle){
        //Rotates module to target angle
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
        SmartDashboard.putNumber(m_pos+"_CurrentAngle", currentAngle);
        SmartDashboard.putNumber(m_pos+"_PV", m_steeringMotor.getEncoder().getPosition());
        SmartDashboard.putNumber(m_pos+"_SP", steering_m_setpoint);
        SmartDashboard.putNumber(m_pos+"_EncoderOutput", absolute_encoder.getOutput());
        SmartDashboard.putNumber(m_pos+"_m_steering_target", m_steering_target);

    }
    
}
