package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberDeploy extends SubsystemBase {
    VictorSPX climberDeployMotor;
    CANSparkMax climberWinchMotor;

    public ClimberDeploy()
    {
        climberDeployMotor = new VictorSPX(Constants.CAN_ID_CLIMBER_DEPLOY);
        
        climberDeployMotor.setNeutralMode(NeutralMode.Brake);

    }
    private void setDeploySpeed(double speed)
    {
        climberDeployMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }
    public void raiseHook()
    {
        setDeploySpeed(1.0);
    }
    public void lowerHook()
    {
        setDeploySpeed(-1.0);
    }
    public void stopHook()
    {
        setDeploySpeed(0);
    }
    public void stop()
    {
        setDeploySpeed(0);
    }

    public void updateDashboard()
    {
        //SmartDashboard.putNumber("Climber Deploy Motor A Current", climberDeployMotor.getStatorCurrent());
    }
}
