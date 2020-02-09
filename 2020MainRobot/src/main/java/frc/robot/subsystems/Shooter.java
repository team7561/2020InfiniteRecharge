package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
public class Shooter extends SubsystemBase {
    CANSparkMax shooterMotorA;
    CANSparkMax shooterMotorB;
    DoubleSolenoid shooterSolenoid;
    private CANPIDController m_pidController;
    private CANEncoder m_encoder;
   // TalonFX shooterA;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setpoint;
  

    public Shooter()
    {
        shooterMotorA = new CANSparkMax(Ports.SHOOTER_A_CANID, MotorType.kBrushless);
        shooterMotorB = new CANSparkMax(Ports.SHOOTER_B_CANID, MotorType.kBrushless);
        //shooterA = new TalonFX(50);
        shooterMotorA.restoreFactoryDefaults();
        shooterMotorB.restoreFactoryDefaults();

        shooterMotorA.setIdleMode(IdleMode.kCoast);
        shooterMotorB.setIdleMode(IdleMode.kCoast);
        shooterMotorA.setSmartCurrentLimit(20);
        shooterMotorB.setSmartCurrentLimit(20);
        m_pidController = shooterMotorA.getPIDController();
        m_encoder = shooterMotorA.getEncoder();

        // PID coefficients
        kP = 5e-5; 
        kI = 2e-7;
        kD = 1e-6; 
        kIz = 0; 
        kFF = 0; 
        setpoint = -3000;
        //kMaxOutput = 1; 
        //kMinOutput = -1;
        kMaxOutput = 0.7; 
        kMinOutput = -0.7;
        maxRPM = 5700;

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
        SmartDashboard.putNumber("Set Point", -setpoint);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        shooterMotorB.follow(ExternalFollower.kFollowerSparkMax, Ports.SHOOTER_A_CANID, true);
        //shooterMotorB.setInverted(true);
        shooterSolenoid = new DoubleSolenoid(Ports.SHOOTER_SOLENOID_CHANNEL_A, Ports.SHOOTER_SOLENOID_CHANNEL_B);
    }

    //Get the Ball
    public void shootBall()
    {
        shooterMotorA.set(-0.075);
        //shooterMotorB.set(-0.075);
        updateDashboard(true);
    }

    public void shootAtSpeed(double setPoint)
    {
        updateDashboard(true);
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        setPoint = -SmartDashboard.getNumber("Set Point", 0);

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
        m_pidController.setReference(setPoint, ControlType.kVelocity);
        //shooterMotorB.set(shooterMotorA.get());
        //SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());

    }
    public double getVelocity()
    {
        return m_encoder.getVelocity();
    }
    public void extendDeflector()
    {
        shooterSolenoid.set(Value.kForward);
    }
    public void retractDeflector()
    {
        shooterSolenoid.set(Value.kReverse);
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
        shooterMotorA.set(0);
    }

    public void updateDashboard(boolean debug)
    {
        if (debug)
        {
            SmartDashboard.putNumber("Shooter A Power", shooterMotorA.getAppliedOutput());
            SmartDashboard.putNumber("Shooter B Power", shooterMotorB.getAppliedOutput());
            SmartDashboard.putNumber("Shooter A Current", shooterMotorA.getOutputCurrent());
            SmartDashboard.putNumber("Shooter B Current", shooterMotorB.getOutputCurrent());
            SmartDashboard.putNumber("Shooter Velocity", m_encoder.getVelocity());
        }
    }
 }