package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
* An example command that uses an example subsystem.
*/
public class Shooter_ShootAtSpeed extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_subsystem;
    private final double m_speed;
    private final boolean m_closeTarget;
    private final boolean m_changeHood;
    
    /**
    * Creates a new ExampleCommand.
    *
    * @param subsystem The subsystem used by this command.
    */
    public Shooter_ShootAtSpeed(Shooter subsystem, double speed){
        m_subsystem = subsystem;
        m_speed = speed;
        m_closeTarget = true;
        m_changeHood = false;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        
    }
    public Shooter_ShootAtSpeed(Shooter subsystem, double speed, boolean changeHood, boolean closeTarget){
        m_subsystem = subsystem;
        m_speed = speed;
        m_closeTarget = closeTarget;
        m_changeHood = changeHood;
        addRequirements(subsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_changeHood) {
            if (m_closeTarget)
            {
                m_subsystem.retractHood();
            }
            else
            {
                m_subsystem.extendHood();
            }
        }
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.shootAtSpeed(m_speed);
        m_subsystem.updateDashboard();
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {        
        return false;
        //return (m_subsystem.getVelocity() > m_speed);
    }
}
