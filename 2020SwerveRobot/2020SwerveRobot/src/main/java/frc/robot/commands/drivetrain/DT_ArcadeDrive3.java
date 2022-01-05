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
    double x = m_x.getAsDouble();
    double y = m_y.getAsDouble();
    double speed = m_speed.getAsDouble();
     
    System.out.println(x);
    System.out.println(y);
    /*x = clip(x);
    y = clip(y);*/
    
    if (m_subsystem.getMode() == SwerveMode.TANK || m_subsystem.getMode() == SwerveMode.TANK_X)
    {
      x = 0;
    }

    m_angle = Math.atan2(y, x)%Math.PI;

    if (x<0 && y<0)
    {
      m_angle += Math.PI;
    }
    
    m_angle = m_angle * 30;

    if (m_angle > 1000)
    {
      m_angle = 0.0;
    }
    m_power = Math.sqrt(Math.pow(x , 2) + Math.pow(y , 2)) * speed;
    System.out.println(m_power);
    m_subsystem.moduleD.setAngle(m_angle);
    m_subsystem.moduleA.setAngle(m_angle);
    m_subsystem.moduleC.setAngle(m_angle);
    m_subsystem.moduleB.setAngle(m_angle);

    drive(m_power, m_power);

    SmartDashboard.putNumber("m_x", m_x.getAsDouble());
    SmartDashboard.putNumber("m_y", m_y.getAsDouble());
    SmartDashboard.putNumber("m_power", m_power);
    SmartDashboard.putNumber("m_twist", m_twist.getAsDouble());
  }

  public void drive(double leftSpeed, double rightSpeed) {
    m_subsystem.moduleD.setVelocity(leftSpeed);
    m_subsystem.moduleC.setVelocity(rightSpeed);
    m_subsystem.moduleA.setVelocity(leftSpeed);
    m_subsystem.moduleB.setVelocity(rightSpeed);
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

  public double clip(double input)
  {
    if (input > 1)
      return 1;
    if (input < 1)
      return -1;
    return input;
  }

}