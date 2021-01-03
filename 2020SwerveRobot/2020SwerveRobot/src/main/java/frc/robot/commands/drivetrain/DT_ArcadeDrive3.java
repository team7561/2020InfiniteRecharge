package frc.robot.commands.drivetrain;

import frc.robot.SwerveMode;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DT_ArcadeDrive3 extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  private DoubleSupplier m_x, m_y, m_twist, m_speed;
  private Double m_angle, m_power;

  public DT_ArcadeDrive3(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier twist, DoubleSupplier speed) {
    m_subsystem = drivetrain;
    m_x = x;
    m_y = y;
    m_speed = speed;
    m_twist = twist;
    
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.updateDashboard();
    double x = m_x.getAsDouble();
    double y = m_y.getAsDouble();
    double speed = m_speed.getAsDouble();
     
    if (x > 1)
    {
      x = 1;
    }
    if (x < -1)
    {
      x = -1;
    }
    if (y > 1)
    {
      y = 1;
    }
    if (y < -1)
    {
      y = -1;
    }
    
    if (m_subsystem.getMode() == SwerveMode.TANK || m_subsystem.getMode() == SwerveMode.TANK_X)
    {
      x = 0;
    }

    m_angle = Math.atan(x/y);

    if ( x  < 0){
        if (y < 0){
            m_angle += Math.PI;
        }
    }
    m_angle = m_angle * 10;
    m_power = Math.sqrt(Math.pow(x , 2) + Math.pow(y , 2)) * speed;

    m_subsystem.moduleBL.setAngle(m_angle);
    m_subsystem.moduleFL.setAngle(m_angle);
    m_subsystem.moduleBR.setAngle(m_angle);
    m_subsystem.moduleFR.setAngle(m_angle);

    drive(m_power, m_power);

    SmartDashboard.putNumber("m_x", m_x.getAsDouble());
    SmartDashboard.putNumber("m_y", m_y.getAsDouble());
    SmartDashboard.putNumber("m_twist", m_twist.getAsDouble());
    m_subsystem.updateDashboard();
  }

  public void drive(double leftSpeed, double rightSpeed) {
    m_subsystem.moduleBL.setVelocity(leftSpeed);
    m_subsystem.moduleBR.setVelocity(rightSpeed);
    m_subsystem.moduleFL.setVelocity(leftSpeed);
    m_subsystem.moduleFR.setVelocity(rightSpeed);
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