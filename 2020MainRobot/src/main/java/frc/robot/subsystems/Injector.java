package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Speeds;

public class Injector extends SubsystemBase {

    CANSparkMax injectorMotor;
    Boolean hasBall;

    public Injector()
    {
        injectorMotor = new CANSparkMax(Ports.INJECTOR_CANID, MotorType.kBrushless);
        injectorMotor.setIdleMode(IdleMode.kCoast);
    }

    //set speed of both intake motors
    private void setSpeed (double speed)
    {
        injectorMotor.set(speed);
    }

    public void transferBall()
    {
        setSpeed(Speeds.INJECTOR_TRANSFER_SPEED);
    }
    //Stops injector
    public void stop()
    {
        setSpeed(Speeds.INJECTOR_STOP_SPEED);
    }
    //Reverses injector
    public void reverse()
    {
        setSpeed(Speeds.INJECTOR_BACKFEED_SPEED);
    }


    public void periodic()
    {
        updateDashboard();
    }
    public void updateDashboard()
    {
        if (Constants.DEBUG)
        {
            SmartDashboard.putNumber("Injector Power", injectorMotor.get());
        }
    }

}
