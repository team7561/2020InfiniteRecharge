package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberWinch extends SubsystemBase {
    VictorSPX climberDeployMotor;
    CANSparkMax climberWinchMotor;

    public ClimberWinch()
    {
        climberWinchMotor = new CANSparkMax(Constants.CAN_ID_CLIMBER_WINCH, MotorType.kBrushless);
        
        climberWinchMotor.setIdleMode(IdleMode.kBrake);

    }
    private void setWinchSpeed(double speed)
    {
        climberWinchMotor.set(speed);
    }
    public void climb()
    {
        setWinchSpeed(-0.5);
    }
    public void climbReverse()
    {
        setWinchSpeed(0.5);
    }
    public void stop()
    {
        setWinchSpeed(0);
    }

    public void updateDashboard()
    {
        //SmartDashboard.putNumber("Climber Deploy Motor A Current", climberDeployMotor.getStatorCurrent());
    }
}
