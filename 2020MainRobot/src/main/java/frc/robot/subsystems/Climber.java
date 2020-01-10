package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.Speeds;

public class Climber implements Subsystem {
    TalonSRX climberMotorA;
    TalonSRX climberMotorB;
    public Climber()
    {
        
    }
    private void setWinchSpeed(double speed)
    {
        //climberMotorA.set(ControlMode.PercentOutput, speed);
        //climberMotorB.set(ControlMode.PercentOutput, -speed);
    }
    public void retractLift()
    {
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
            SmartDashboard.putNumber("Climber Motor A Current", climberMotorA.getOutputCurrent());
            SmartDashboard.putNumber("Climber Motor B Speed", climberMotorB.getMotorOutputPercent());
            SmartDashboard.putNumber("Climber Motor B Current", climberMotorB.getOutputCurrent());
        }


    }

}
