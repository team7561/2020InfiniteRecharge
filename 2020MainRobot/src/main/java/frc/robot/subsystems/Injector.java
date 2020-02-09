package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Speeds;

public class Injector extends SubsystemBase {

    CANSparkMax injectorrMotor;
    Boolean hasBall;

    public Injector()
    {
        injectorrMotor = new CANSparkMax(Ports.INJECTOR_CANID, MotorType.kBrushless);
    }

    //set speed of both intake motors
    private void setSpeed (double speed)
    {
        injectorrMotor.set(speed);
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


    public void updateDashboard(boolean debug)
    {
        if (debug)
        {
            SmartDashboard.putNumber("Injector Power", injectorrMotor.get());
        }
    }




}
