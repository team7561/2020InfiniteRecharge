package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Speeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ControlPanelManipulator extends SubsystemBase {

    ColourSensor colourSensor;
    VictorSPX colourWheelRotateMotor;
    DoubleSolenoid colourWheelSolenoid;

    public ControlPanelManipulator()
    {
        colourSensor = new ColourSensor();
        colourWheelRotateMotor = new VictorSPX(Ports.COLOUR_WHEEL_ROTATE_CANID);
        colourWheelSolenoid = new DoubleSolenoid(Ports.CPM_SOLENOID_CHANNEL_A, Ports.CPM_SOLENOID_CHANNEL_B);
        
    }
    private void setSpeed(double speed)
    {
        colourWheelRotateMotor.set(ControlMode.PercentOutput, speed);
    }
    public String detectColour()
    {
        return colourSensor.robotPeriodic();
    }
    public void rotate()
    {
        setSpeed(Speeds.ROTATION_CONTROL_SPEED);
    }
    public void stop()
    {
        setSpeed(0);
    }
    public void extend()
    {
        colourWheelSolenoid.set(Value.kForward);
    }
    public void retract()
    {
        colourWheelSolenoid.set(Value.kReverse);
    }
}
