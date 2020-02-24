package frc.robot.commands.drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionController;
import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * An example command that uses an example subsystem.
 */
public class TurnToVisionAngle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  private final VisionController m_vision_subsystem;
  private final double m_speed;
  private double m_targetAngle;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnToVisionAngle(Drivetrain subsystem, VisionController vision_subsystem, double speed){
    m_subsystem = subsystem;
    m_vision_subsystem = vision_subsystem;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting turn to vision angle");
    System.out.println("Turn to vision angle called");
    m_vision_subsystem.turnOnLED();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vision_subsystem.turnOnLED();
    System.out.println("Turning to vision angle");
    m_targetAngle = m_vision_subsystem.get_tx();
    System.out.println("tx = " + m_targetAngle);
    
    if (m_targetAngle > 10) {
      m_subsystem.drive(m_speed, -m_speed);
    }
    else if (m_targetAngle < -10) {
      m_subsystem.drive(-m_speed, m_speed);
      return;
    }
    else if (m_targetAngle > 5) {
      m_subsystem.drive(m_speed/2, -m_speed/2);
    }
    else if (m_targetAngle < -5) {
      m_subsystem.drive(-m_speed/2, m_speed/2);
    }
    else if (m_targetAngle > 3) {
      m_subsystem.drive(m_speed/3, -m_speed/3);
    }
    else if (m_targetAngle < -3) {
      m_subsystem.drive(-m_speed/3, m_speed/3);
    }
    else if (m_targetAngle > 1) {
      m_subsystem.drive(m_speed/4, -m_speed/4);
    }
    else if (m_targetAngle < -1) {
      m_subsystem.drive(-m_speed/4, m_speed/4);
    }
    else{
      System.out.println("At vision target)");
      m_subsystem.drive(0, 0);
    }
    m_subsystem.updateDashboard();
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Turning to vision target finished");  
    m_vision_subsystem.turnOffLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {        
    //return (m_targetAngle <= Constants.ANGLE_TOLERANCE);
    return false;
  }
}
