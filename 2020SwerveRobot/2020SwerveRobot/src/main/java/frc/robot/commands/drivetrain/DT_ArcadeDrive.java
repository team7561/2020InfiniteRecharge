package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DT_ArcadeDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  private Double m_x, m_y, m_speed;
  private Double m_angle, m_power;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DT_ArcadeDrive(Drivetrain subsystem, Double x, Double y, Double speed) {
    m_subsystem = subsystem;
    m_x = x;
    m_y = y;
    m_speed = speed;
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
    m_subsystem.updateDashboard();

    m_angle = Math.atan(m_x / m_y );

    if ( m_x  < 0){
        if (m_y  >= 0){
            m_angle += Math.PI;
        }
    }

    m_power = Math.sqrt(Math.pow(m_x , 2) + Math.pow(m_y , 2)) * m_speed ;

    m_subsystem.moduleBL.setAngle(m_angle);
    m_subsystem.moduleFL.setAngle(m_angle);
    m_subsystem.moduleBR.setAngle(m_angle);
    m_subsystem.moduleFR.setAngle(m_angle);

    double m_power = 0.25;
    m_subsystem.moduleBL.setSpeed(m_power);
    m_subsystem.moduleBR.setSpeed(m_power);
    m_subsystem.moduleFR.setSpeed(m_power);
    m_subsystem.moduleFL.setSpeed(m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
