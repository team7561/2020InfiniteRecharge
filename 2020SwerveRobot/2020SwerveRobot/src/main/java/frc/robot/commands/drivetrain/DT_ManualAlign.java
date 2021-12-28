package frc.robot.commands.drivetrain;

import frc.robot.SwerveMode;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command that uses an example subsystem.
 */
public class DT_ManualAlign extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  private DoubleSupplier m_x, m_y, m_twist, m_speed;
  private double abs_x, abs_y, target_angle, m_power, current_angle = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param drivetrain The subsystem used by this command.
   */
  public DT_ManualAlign(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier twist, DoubleSupplier speed) {
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
    abs_x = Math.abs(m_x.getAsDouble());
    abs_y = Math.abs(m_y.getAsDouble());

    if (m_subsystem.moduleBL.getAngleOffset() != SmartDashboard.getNumber("BL_Offset_Angle", Constants.SWERVE_BL_OFFSET_ANGLE))
    {
        m_subsystem.moduleBL.setAngleOffset(SmartDashboard.getNumber("BL_Offset_Angle", Constants.SWERVE_BL_OFFSET_ANGLE));
    }
    if (m_subsystem.moduleBR.getAngleOffset() != SmartDashboard.getNumber("BR_Offset_Angle", Constants.SWERVE_BR_OFFSET_ANGLE))
    {
        m_subsystem.moduleBR.setAngleOffset(SmartDashboard.getNumber("BR_Offset_Angle", Constants.SWERVE_BR_OFFSET_ANGLE));
    }
    if (m_subsystem.moduleFL.getAngleOffset() != SmartDashboard.getNumber("FL_Offset_Angle", Constants.SWERVE_FL_OFFSET_ANGLE))
    {
      m_subsystem.moduleFL.setAngleOffset(SmartDashboard.getNumber("FL_Offset_Angle", Constants.SWERVE_FL_OFFSET_ANGLE));
    }
    if (m_subsystem.moduleFR.getAngleOffset() != SmartDashboard.getNumber("FR_Offset_Angle", Constants.SWERVE_FR_OFFSET_ANGLE))
    {
      m_subsystem.moduleFR.setAngleOffset(SmartDashboard.getNumber("FR_Offset_Angle", Constants.SWERVE_FR_OFFSET_ANGLE));
    }
    
/*
    if( abs_x != 0 && abs_y != 0 ){
      if (m_y.getAsDouble() > 0 ){
        target_angle = Math.PI/2 + Math.atan(abs_y/abs_x);
      } else {
        target_angle = Math.PI/2 - Math.atan(abs_y/abs_x);
      }
  
      if (m_x.getAsDouble() < 0){
        target_angle = 2*Math.PI - target_angle;
      }
    }
*/
  m_power = Math.sqrt(Math.pow(abs_x , 2) + Math.pow(abs_y , 2)) * m_speed.getAsDouble();
  if (m_subsystem.getMode() == SwerveMode.SPIN){
    m_power = -1 * m_twist.getAsDouble() * m_speed.getAsDouble();
  }
  
//  target_angle = Math.atan2(m_y.getAsDouble(), m_x.getAsDouble())+Math.PI;
    target_angle = 90;

    System.out.println(target_angle);

    SmartDashboard.putNumber("current_angle", current_angle);
    SmartDashboard.putNumber("measured_angle", m_subsystem.moduleBL.getAngle());
    SmartDashboard.putNumber("target_angle", target_angle);

    m_subsystem.setAngle(target_angle);
    m_subsystem.updateDashboard();
    drive(0.1, 0.1);
  }

  public void drive(double leftSpeed, double rightSpeed) {
    m_subsystem.moduleBL.setVelocity(leftSpeed);
    m_subsystem.moduleBR.setVelocity(-rightSpeed);
    m_subsystem.moduleFL.setVelocity(leftSpeed);
    m_subsystem.moduleFR.setVelocity(-rightSpeed);
  }
  public void arcadeDrive(double x, double y, double speed, boolean inverted) {
    if (m_subsystem.getMode() == SwerveMode.TANK || m_subsystem.getMode() == SwerveMode.TANK_X)
    {
      x = 0;
    }

    double right = (-y - x)*speed;
    double left = (- (y - x))*speed;
    if (left > 1) {
        left = 1;
    }
    if (right > 1) {
        right = 1;
    }
    if (inverted == true) {
        drive(-left*10, -right*10);
    }
    else
    {
        drive(left*10, right*10);
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
