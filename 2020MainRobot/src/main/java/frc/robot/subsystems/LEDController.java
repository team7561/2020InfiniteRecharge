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
    
}
