package frc.robot.commands.drivetrain;

import frc.robot.SwerveMode;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command that uses an example subsystem.
 */
public class DT_DriveTime extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  Timer timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param drivetrain The subsystem used by this command.
   */
  public DT_DriveTime(Drivetrain drivetrain) {
    m_subsystem = drivetrain;
    timer = new Timer();
    
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting auto");
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setAngle(0);
    drive(1000, 1000);
    System.out.println("Auto Drive");
    m_subsystem.updateDashboard();
  }

  public void drive(double leftSpeed, double rightSpeed) {
    m_subsystem.moduleBL.setVelocity(leftSpeed);
    m_subsystem.moduleBR.setVelocity(-rightSpeed);
    m_subsystem.moduleFL.setVelocity(leftSpeed);
    m_subsystem.moduleFR.setVelocity(-rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>1;
  }
}
