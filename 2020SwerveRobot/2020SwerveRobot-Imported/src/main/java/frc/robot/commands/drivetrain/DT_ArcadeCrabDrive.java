package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command that uses an example subsystem.
 */
public class DT_ArcadeCrabDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  private DoubleSupplier m_x, m_y, m_twist, m_speed;
  private Double m_angle, m_power;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DT_ArcadeCrabDrive(Drivetrain subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier twist, DoubleSupplier speed) {
    m_subsystem = subsystem;
    m_x = x;
    m_y = y;
    m_twist = twist;
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
    arcadeDrive(m_y.getAsDouble(), 0, m_speed.getAsDouble(), false);
    SmartDashboard.putNumber("m_x", m_x.getAsDouble());
    SmartDashboard.putNumber("m_y", m_y.getAsDouble());
    m_subsystem.updateDashboard();
    m_subsystem.setAngle(m_twist.getAsDouble()*20);

  }
  public void drive(double leftSpeed, double rightSpeed) {
    m_subsystem.moduleBL.setSpeed(leftSpeed);
    m_subsystem.moduleBR.setSpeed(-rightSpeed);
    m_subsystem.moduleFL.setSpeed(leftSpeed);
    m_subsystem.moduleFR.setSpeed(-rightSpeed);
  }
  public void arcadeDrive(double x, double y, double speed, boolean inverted) {
    //x = x * Math.abs(x) * speed;
    //y = y * Math.abs(y) * speed;

    double right = (-y - x)*speed;
    double left = (- (y - x))*speed;
    if (left > 1) {
        left = 1;
    }
    if (right > 1) {
        right = 1;
    }
    if (inverted == true) {
        drive(-left, -right);
    }
    else
    {
        drive(left, right);
    }
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