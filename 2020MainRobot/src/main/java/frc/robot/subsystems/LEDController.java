package frc.robot.subsystems;
import frc.robot.Ports;
import edu.wpi.first.wpilibj.Spark;
public class LEDController {
    Spark blinkin = new Spark(Ports.LED_CONTROLLER_CHANNEL);

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
    
}
