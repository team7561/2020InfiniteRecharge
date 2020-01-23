package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Speeds;

public class Climber extends SubsystemBase {
    TalonFX climberMotorA;
    TalonFX climberMotorB;
    TalonSRX climberDeployMotor;
    DigitalInput climberHookExtended;
    public Climber()
    {
        climberMotorA = new TalonFX(Ports.CLIMB_WINCH_A_CANID);
        climberMotorB = new TalonFX(Ports.CLIMB_WINCH_B_CANID);
        climberMotorB.follow(climberMotorA);
        climberDeployMotor = new TalonSRX(Ports.CLIMB_DEPLOY_A_CANID);
        climberHookExtended = new DigitalInput(Ports.CLIMBER_HOOK_DEPLOY_LIMIT_SWITCH_CHANNEL);
    }
    private void setWinchSpeed(double speed)
    {
        climberMotorA.set(ControlMode.PercentOutput, speed);
    }
    public void raiseHook()
    {
        if (!climberHookExtended.get())
        {
            climberDeployMotor.set(ControlMode.PercentOutput, Speeds.CLIMBER_HOOK_RAISE_SPEED);
        }
    }
    public void lowerHook()
    {
        climberDeployMotor.set(ControlMode.PercentOutput, Speeds.CLIMBER_HOOK_LOWER_SPEED);
    }
    public void stopClimbing()
    {
        setWinchSpeed(Speeds.CLIMBER_STOP_SPEED);
    }
    public void stop()
    {
        setWinchSpeed(Speeds.CLIMBER_STOP_SPEED);
        climberDeployMotor.set(ControlMode.PercentOutput, 0);
    }
    public void updateDashboard(boolean debug)
    {
        if (debug)
            {
            SmartDashboard.putNumber("Climber Motor A Speed", climberMotorA.getMotorOutputPercent());
            SmartDashboard.putNumber("Climber Motor A Current", climberMotorA.getOutputCurrent());
            SmartDashboard.putNumber("Climber Motor B Speed", climberMotorB.getMotorOutputPercent());
            SmartDashboard.putNumber("Climber Motor B Current", climberMotorB.getOutputCurrent());
            SmartDashboard.putNumber("Climber Deploy Motor Current", climberDeployMotor.getOutputCurrent());
            SmartDashboard.putNumber("Climber Deploy Motor Speed", climberDeployMotor.getMotorOutputPercent());
        }


    }

}
