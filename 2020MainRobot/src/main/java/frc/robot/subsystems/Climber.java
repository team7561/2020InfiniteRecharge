package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.Speeds;

public class Climber extends SubsystemBase {
    TalonSRX climberMotorA;
    VictorSPX climberMotorB;
    TalonSRX climberDeployMotor;
    public Climber()
    {
        climberMotorA = new TalonSRX(Ports.CLIMB_WINCH_A_CANID);
        climberMotorB = new VictorSPX(Ports.CLIMB_WINCH_B_CANID);
        climberMotorB.follow(climberMotorA);

        climberDeployMotor = new TalonSRX(Ports.CLIMB_DEPLOY_A_CANID);
    }
    private void setWinchSpeed(double speed)
    {
        climberMotorA.set(ControlMode.PercentOutput, speed);
    }
    public void raiseHook()
    {
        climberDeployMotor.set(ControlMode.PercentOutput, Speeds.CLIMBER_HOOK_RAISE_SPEED);
    }
    public void lowerHook()
    {
        climberDeployMotor.set(ControlMode.PercentOutput, Speeds.CLIMBER_HOOK_LOWER_SPEED);
    }
    public void stopClimbing()
    {
        setWinchSpeed(Speeds.CLIMBER_STOP_SPEED);
    }
    public void updateDashboard(boolean debug)
    {
        if (debug)
            {
            SmartDashboard.putNumber("Climber Motor A Speed", climberMotorA.getMotorOutputPercent());
            SmartDashboard.putNumber("Climber Motor Current", climberMotorA.getOutputCurrent());
            SmartDashboard.putNumber("Climber Motor B Speed", climberMotorB.getMotorOutputPercent());
        }


    }

}
