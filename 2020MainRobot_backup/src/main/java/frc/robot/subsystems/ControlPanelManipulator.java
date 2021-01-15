package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Speeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlPanelManipulator extends SubsystemBase {

    public ColourSensor colourSensor;
    VictorSP colourWheelRotateMotor;
    DoubleSolenoid colourWheelSolenoid;

    public ControlPanelManipulator()
    {
        colourSensor = new ColourSensor();
        colourWheelRotateMotor = new VictorSP(Ports.COLOUR_WHEEL_ROTATE_CHANNEL);
        colourWheelSolenoid = new DoubleSolenoid(Ports.CPM_SOLENOID_CHANNEL_A, Ports.CPM_SOLENOID_CHANNEL_B);   
    }
    private void setSpeed(double speed)
    {
        colourWheelRotateMotor.set(speed);
    }
    public String detectColour()
    {
        return colourSensor.periodic();
    }
    public void rotateLeft()
    {
        setSpeed(Speeds.ROTATION_CONTROL_SPEED);
    }
    public void rotateRight()
    {
        setSpeed(-Speeds.ROTATION_CONTROL_SPEED);
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
    public void updateDashboard()
    {
        SmartDashboard.putNumber("Colour Wheel Motor Speed", colourWheelRotateMotor.get());
    }
}
