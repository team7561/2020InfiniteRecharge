package frc.robot.subsystems;
import frc.robot.Ports;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase{
    Spark blinkin = new Spark(Ports.LED_CONTROLLER_CHANNEL);
    ColourSensor colourSensor;

    public void Rainbow(){
        blinkin.set(-0.91);
    }
    
    public void Red(){
        blinkin.set(0.61);
    }

    public void Green(){
        blinkin.set(0.77);
    }

    public void Yellow(){
        blinkin.set(0.69);
    }

    public void Blue(){
        blinkin.set(0.83);
    }
    public String detectColour()
    {
        return colourSensor.periodic();
    }
    
}
