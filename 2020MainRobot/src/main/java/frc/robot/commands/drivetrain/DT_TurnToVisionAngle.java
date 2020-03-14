package frc.robot.commands.drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionController;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


/**
* An example command that uses an example subsystem.
*/
public class DT_TurnToVisionAngle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  private final VisionController m_vision_subsystem;
  private DoubleSupplier m_speedSupplier;
  private double m_targetAngle, m_speed;
  private Timer timer, timerFinished;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DT_TurnToVisionAngle(Drivetrain subsystem, VisionController vision_subsystem, DoubleSupplier speedSupplier){
    m_subsystem = subsystem;
    m_vision_subsystem = vision_subsystem;
    m_speedSupplier = speedSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    timer = new Timer();
    timerFinished = new Timer();
  }

  // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Starting turn to vision angle");
        System.out.println("Turn to vision angle called");
        m_vision_subsystem.turnOnLED();
        timer.start();
        SmartDashboard.putBoolean("Turn to Vision Angle is finished: ", false);
        
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_speed = (m_speedSupplier.getAsDouble()/4);
        SmartDashboard.putNumber("m_speed", m_speed);
        m_vision_subsystem.turnOnLED();
        //System.out.println("Turning to vision angle");
        m_targetAngle = m_vision_subsystem.get_tx();
        System.out.println("tx = " + m_targetAngle);
        double errorSpeed = motorSpeedForError(m_targetAngle)/20+0.5*(m_targetAngle/Math.abs(m_targetAngle));
        SmartDashboard.putNumber("m_speed", m_speed);
        SmartDashboard.putNumber("m_targetAngle", m_targetAngle);
        SmartDashboard.putNumber("errorSpeed", errorSpeed);
        
        if (Math.abs(m_targetAngle) > Constants.ANGLE_TOLERANCE) {
            m_subsystem.drive(m_speed*errorSpeed, -m_speed*errorSpeed);
        }
        else {
            System.out.println("At vision target)");
            m_subsystem.drive(0, 0);
        }
        m_subsystem.updateDashboard();
    }
    static double motorSpeedForError(double error) {
        if (error < 0) {
            if (error < -16) {
                if (error < -20)
                return error;
                return -20;
            }
            if (error < -12)
            return error - 4;
            return (2 * error) + 8;
        }
        if (error < 16) {
            if (error < 12)
            return 8;
            return error - 4;
        }
        if (error < 20)
        return (2 * error) - 20;
        return error;
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Turning to vision target finished");  
        m_vision_subsystem.turnOffLED();
        m_subsystem.drive(0, 0);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {    
        boolean isAtAngle = Math.abs(m_targetAngle) <= Constants.ANGLE_TOLERANCE;
        if (isAtAngle)
        {
            timerFinished.start();
        }
        else {
            timerFinished.reset();
            timerFinished.stop();
        }
        if (timer.get()<10.3)
        {
            return false;
        }
        if (timerFinished.get()>12)
        {
            SmartDashboard.putBoolean("Turn to Vision Angle is finished: ", true);
            //return false;
            return true;
        }
        return false;
    }
}
