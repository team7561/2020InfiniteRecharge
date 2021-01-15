package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
public class Shooter extends SubsystemBase {
    CANSparkMax shooterMotorA;
    CANSparkMax shooterMotorB;
    TalonSRX shooterHood;

    private CANPIDController m_pidController;
    private CANEncoder m_encoder;
    private boolean shooting, hood_auto;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, m_setpoint;

    public Shooter()
    {

        shooterMotorA = new CANSparkMax(Ports.SHOOTER_A_CANID, MotorType.kBrushless);
        shooterMotorB = new CANSparkMax(Ports.SHOOTER_B_CANID, MotorType.kBrushless);
        shooterHood = new TalonSRX(Ports.SHOOTER_HOOD_CANID);
        
        shooterMotorA.restoreFactoryDefaults();
        shooterMotorB.restoreFactoryDefaults();

        shooterMotorA.setIdleMode(IdleMode.kCoast);
        shooterMotorB.setIdleMode(IdleMode.kCoast);
        
        shooterMotorA.setSmartCurrentLimit(45);
        shooterMotorB.setSmartCurrentLimit(45);
        
        m_pidController = shooterMotorA.getPIDController();
        m_encoder = shooterMotorA.getEncoder();

        shooterHood.configFactoryDefault();
        shooterHood.configPeakCurrentLimit(2);
        shooterHood.configPeakCurrentDuration(200);
        shooterHood.configContinuousCurrentLimit(3);
        shooterHood.enableCurrentLimit(true);
        shooterHood.setNeutralMode(NeutralMode.Brake);
        shooterHood.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        shooterHood.setSensorPhase(true);

        hood_auto = false;
        shooting = false; 

        // PID coefficients
        kP = 0.003; 
        kI = 0.000002;
        kD = 0.000004; 
        kIz = 500; // Error process value must be within before I is used.
        kFF = 0; 
        m_setpoint = 2500;
        kMaxOutput = 0.85; 
        kMinOutput = -0.85;
        maxRPM = 4500;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Set Point", m_setpoint);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        shooterMotorB.follow(ExternalFollower.kFollowerSparkMax, Ports.SHOOTER_A_CANID, true);
    }

    public void shootAtSpeed(double setPoint)
    {
        shooting = true;
    }
    public void periodic()
    {
        if (shooting)
        {
            System.out.println("Shooting at speed: " + m_setpoint);
            // read PID coefficients from SmartDashboard
            double p = SmartDashboard.getNumber("P Gain", 0);
            double i = SmartDashboard.getNumber("I Gain", 0);
            double d = SmartDashboard.getNumber("D Gain", 0);
            double iz = SmartDashboard.getNumber("I Zone", 0);
            double ff = SmartDashboard.getNumber("Feed Forward", 0);
            double max = SmartDashboard.getNumber("Max Output", 0);
            double min = SmartDashboard.getNumber("Min Output", 0);
            m_setpoint = SmartDashboard.getNumber("Set Point", 0);

            // if PID coefficients on SmartDashboard have changed, write new values to controller
            if((p != kP)) { m_pidController.setP(p); kP = p; }
            if((i != kI)) { m_pidController.setI(i); kI = i; }
            if((d != kD)) { m_pidController.setD(d); kD = d; }
            if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
            if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
            if((max != kMaxOutput) || (min != kMinOutput)) { 
            m_pidController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
            }
            m_pidController.setReference(m_setpoint, ControlType.kVelocity);
            //shooterMotorB.set(shooterMotorA.get());
            //SmartDashboard.putNumber("SetPoint", setPoint);
            SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
        }
        else
        {
            shooterMotorA.set(0);
        }
        if (hood_auto)
        {

        }
    }
    public double getVelocity()
    {
        return m_encoder.getVelocity();
    }
    public void extendHood()
    {
        shooterHood.set(ControlMode.PercentOutput, 0.25);
    }
    public void retractHood()
    {
        shooterHood.set(ControlMode.PercentOutput, -0.25);
    }
    public void stopHood()
    {
        shooterHood.set(ControlMode.PercentOutput, 0);
    }
    public void setSetpoint(double setPoint)
    {
        m_setpoint = setPoint;
    }

    //Ejects the Ball slow
   /* public void ejectBallSlow()
    {
        intakeSpeed(-0.4);
    }
   */
    //Stops shooter
    public void stop()
    {
        shooting = false;
        stopHood();
    }

    public void updateDashboard()
    {
        if (Constants.DEBUG_SHOOTER)
        {
            SmartDashboard.putNumber("Shooter A Power", shooterMotorA.getAppliedOutput());
            SmartDashboard.putNumber("Shooter B Power", shooterMotorB.getAppliedOutput());
            SmartDashboard.putNumber("Shooter A Current", shooterMotorA.getOutputCurrent());
            SmartDashboard.putNumber("Shooter B Current", shooterMotorB.getOutputCurrent());
            SmartDashboard.putNumber("Shooter Velocity", m_encoder.getVelocity());
            SmartDashboard.putNumber("Shooter Hood Position", m_encoder.getPosition());
            SmartDashboard.putNumber("Shooter Hood Voltage", shooterHood.getMotorOutputPercent());
        }
    }
 }