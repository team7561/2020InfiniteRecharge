package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DriveToDistance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  private final double m_speed, m_distance, m_initial_Pulses;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveToDistance(Drivetrain subsystem, double speed, double distance){
    m_subsystem = subsystem;
    m_speed = speed;
    m_distance = distance;
    m_initial_Pulses = m_subsystem.getLeftEncoder();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_subsystem.arcadeDrive(0, 1, m_speed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {        
        return (m_subsystem.getLeftEncoder() > m_distance);
  }
}
