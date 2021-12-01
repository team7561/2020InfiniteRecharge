package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    CANSparkMax shooterMotorA;
    CANSparkMax shooterMotorB;

    private CANEncoder m_flywheel_encoder;


    public Shooter()
    {
        shooterMotorA = new CANSparkMax(Constants.CAN_ID_SHOOTER_A, MotorType.kBrushless);
        shooterMotorB = new CANSparkMax(Constants.CAN_ID_SHOOTER_A, MotorType.kBrushless);
        
        shooterMotorA.restoreFactoryDefaults();
        shooterMotorB.restoreFactoryDefaults();

        shooterMotorA.setIdleMode(IdleMode.kCoast);
        shooterMotorB.setIdleMode(IdleMode.kCoast);
        
        shooterMotorA.setSmartCurrentLimit(20);
        shooterMotorB.setSmartCurrentLimit(20);
    
    }

    public void startFlywheel()
    {
        shooterMotorA.set(0.2);
        shooterMotorB.set(-0.2);
        System.out.println("Shooting");
    }
    public double getVelocity()
    {
        return m_flywheel_encoder.getVelocity();
    }
    public void periodic()
    {
        shooterMotorA.set(0.1);
        shooterMotorA.set(-0.1);
    }
    //Stops shooter
    public void stop()
    {
        shooterMotorB.set(0);
    }
    public void updateDashboard()
    {
        if (true)
        {
            //SmartDashboard.putNumber("Shooter A Power", shooterMotorA.getAppliedOutput());
            SmartDashboard.putNumber("Shooter B Power", shooterMotorB.getAppliedOutput());
            //SmartDashboard.putNumber("Shooter A Current", shooterMotorA.getOutputCurrent());
            SmartDashboard.putNumber("Shooter B Current", shooterMotorB.getOutputCurrent());;
        }
    }
}
