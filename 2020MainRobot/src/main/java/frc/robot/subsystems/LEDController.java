package frc.robot.subsystems;
import frc.robot.Ports;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase{

    Spark blinkin;
    public LEDController() {
        blinkin = new Spark(Ports.LED_CONTROLLER_CHANNEL);
        SmartDashboard.putNumber("LED Value", 0.83);
    }

    public void periodic(){
        blinkin.set(SmartDashboard.getNumber("LED Value", 0.83));
        SmartDashboard.putNumber("LED Value", blinkin.get());
    }

    public void Rainbow(){
        blinkin.set(-0.91);
    }
    public void RainbowWithGlitter(){
        blinkin.set(-0.89);
    }
    public void ShotBlue(){
        blinkin.set(-0.83);
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

    public void White(){
        blinkin.set(0.93);
    }

    public void Lightchase(){
        blinkin.set(-0.29);
    }
    
}
