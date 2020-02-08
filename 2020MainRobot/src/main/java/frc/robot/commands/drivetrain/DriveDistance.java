package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * An example command that uses an example subsystem.
 */
public class DriveDistance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  private final double m_speed;
  private final double m_distance;
  private double m_difference; //difference between final distance and distance robot travelled


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public DriveDistance(Drivetrain subsystem, double speed, double distance){
    m_subsystem = subsystem;
    m_speed = speed;
    m_distance = distance;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_difference = m_distance - m_subsystem.getRightEncoder();
    if (m_difference > m_distance/2){
      m_subsystem.drive(m_speed/2, m_speed/2); 
    }
    if (m_difference > m_distance/4){
      m_subsystem.drive(m_speed/4, m_speed/4); 
    }
    else{
      m_subsystem.drive(m_speed, m_speed);
    }
    m_subsystem.updateDashboard(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_subsystem.getRightEncoder() >= m_distance);
  }
}
