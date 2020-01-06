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
    VictorSPX climberVacuumMotor;
    DoubleSolenoid climberSolenoid;
    DigitalInput climberLimitDigitalInput;
    public Climber()
    {
        climberSolenoid = new DoubleSolenoid(Ports.CLIMBER_SOLENOID_CHANNEL_A, Ports.CLIMBER_SOLENOID_CHANNEL_B);
    }
    private void setWinchSpeed(double speed)
    {
        //climberMotorA.set(ControlMode.PercentOutput, speed);
        //climberMotorB.set(ControlMode.PercentOutput, -speed);
    }
    public void retractLift()
    {
        climberSolenoid.set(DoubleSolenoid.Value.kForward);
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
            SmartDashboard.putBoolean("Climber Limit Switch", climberLimitDigitalInput.get());
            SmartDashboard.putString("Climber Solenoid", climberSolenoid.get().toString());
        }


    }

}
