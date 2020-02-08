package frc.robot.commands.controlpanelmanipulator;

import frc.robot.subsystems.ControlPanelManipulator;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class SpinToColour extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ControlPanelManipulator m_subsystem;
  private String m_currentColour;
  private String m_desiredColour;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SpinToColour(ControlPanelManipulator subsystem, String desiredColour) {
    m_subsystem = subsystem;
    m_desiredColour = desiredColour;
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
    m_currentColour = m_subsystem.detectColour();
    m_subsystem.rotate();
    m_subsystem.updateDashboard(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_currentColour == m_desiredColour);
  }
}
