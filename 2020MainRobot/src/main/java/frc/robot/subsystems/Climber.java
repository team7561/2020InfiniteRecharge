package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Speeds;

public class Climber extends SubsystemBase {
    TalonFX climberMotorA;
    TalonFX climberMotorB;
    VictorSPX climberDeployMotorA;
    VictorSPX climberDeployMotorB;
    DigitalInput climberHookExtended;
    public Climber()
    {
        //climberMotorA = new TalonFX(Ports.CLIMB_WINCH_A_CANID);
        //climberMotorB = new TalonFX(Ports.CLIMB_WINCH_B_CANID);
        /*climberMotorA.configFactoryDefault();
        climberMotorB.configFactoryDefault();
        climberMotorB.follow(climberMotorA);*/
        /*climberMotorA.configContinuousCurrentLimit(10, 0);
        talon.configPeakCurrentLimit(15, 0);
        talon.configPeakCurrentDuration(100, 0);
        talon.enableCurrentLimit(true);*/

        climberDeployMotorA = new VictorSPX(Ports.CLIMB_DEPLOY_A_CANID);
        climberDeployMotorB = new VictorSPX(Ports.CLIMB_DEPLOY_B_CANID);
        /*climberDeployMotorA.configFactoryDefault();
        climberDeployMotorB.configFactoryDefault();
        climberDeployMotorB.follow(climberDeployMotorA);*/
        climberHookExtended = new DigitalInput(Ports.CLIMBER_HOOK_DEPLOY_LIMIT_SWITCH_CHANNEL);
    }
    private void setWinchSpeed(double speed)
    {
        //climberMotorA.set(ControlMode.PercentOutput, speed);
    }
    public void raiseHook()
    {
        climberDeployMotorA.set(ControlMode.PercentOutput, Speeds.CLIMBER_HOOK_RAISE_SPEED);
    }
    public void lowerHook()
    {
        climberDeployMotorA.set(ControlMode.PercentOutput, Speeds.CLIMBER_HOOK_LOWER_SPEED);
    }
    public void stopClimbing()
    {
        setWinchSpeed(Speeds.CLIMBER_STOP_SPEED);
    }
    public void stop()
    {
        setWinchSpeed(Speeds.CLIMBER_STOP_SPEED);
        climberDeployMotorA.set(ControlMode.PercentOutput, 0);
    }
    public void updateDashboard()
    {
        if (Constants.DEBUG)
            {
            /*SmartDashboard.putNumber("Climber Motor A Speed", climberMotorA.getMotorOutputPercent());
            SmartDashboard.putNumber("Climber Motor A Current", climberMotorA.getStatorCurrent());
            SmartDashboard.putNumber("Climber Motor B Speed", climberMotorB.getMotorOutputPercent());
            SmartDashboard.putNumber("Climber Motor B Current", climberMotorB.getStatorCurrent());*/
            //SmartDashboard.putNumber("Climber Deploy Motor A Current", climberDeployMotorA.getStatorCurrent());
            //SmartDashboard.putNumber("Climber Deploy Motor B Current", climberDeployMotorB.getStatorCurrent());
            SmartDashboard.putNumber("Climber Deploy Motor A Speed", climberDeployMotorA.getMotorOutputPercent());
            //SmartDashboard.putNumber("Climber Deploy Motor B Speed", climberDeployMotorB.getMotorOutputPercent());
        }


    }

}
