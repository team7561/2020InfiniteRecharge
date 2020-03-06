package frc.robot.commands.drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionController;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * An example command that uses an example subsystem.
 */
public class TurnToVisionAngle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_subsystem;
  private final VisionController m_vision_subsystem;
  private DoubleSupplier m_speedSupplier;
  private double m_targetAngle, m_speed;
  private Timer timer, timerFinished;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnToVisionAngle(Drivetrain subsystem, VisionController vision_subsystem, DoubleSupplier speedSupplier){
    m_subsystem = subsystem;
    m_vision_subsystem = vision_subsystem;
    m_speedSupplier = speedSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    timer = new Timer();
    timerFinished = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting turn to vision angle");
    System.out.println("Turn to vision angle called");
    m_vision_subsystem.turnOnLED();
    timer.start();
    SmartDashboard.putBoolean("Turn to Vision Angle is finished: ", false);
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_speed = 0.4;   //(m_speedSupplier.getAsDouble()/4)+0.2;
    SmartDashboard.putNumber("m_speed", m_speed);
    m_vision_subsystem.turnOnLED();
    System.out.println("Turning to vision angle");
    m_targetAngle = m_vision_subsystem.get_tx();
    System.out.println("tx = " + m_targetAngle);
    double errorSpeed = m_targetAngle/25 + 0.1 * (m_targetAngle/Math.abs(m_targetAngle));
    SmartDashboard.putNumber("m_speed", m_speed);
    SmartDashboard.putNumber("m_targetAngle", m_targetAngle);
    SmartDashboard.putNumber("errorSpeed", errorSpeed);
    
    if (Math.abs(m_targetAngle) > Constants.ANGLE_TOLERANCE) {
      m_subsystem.drive(m_speed*errorSpeed, -m_speed*errorSpeed);
    }
    else {
      System.out.println("At vision target)");
      m_subsystem.drive(0, 0);
    }
    m_subsystem.updateDashboard();
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Turning to vision target finished");  
    m_vision_subsystem.turnOffLED();
    m_subsystem.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {    
    boolean isFinished = Math.abs(m_targetAngle) <= Constants.ANGLE_TOLERANCE;
    if (isFinished)
    {
      timerFinished.start();
    }
    if (timer.get()<0.3)
    {
      return false;
    }
    else
    {
      SmartDashboard.putBoolean("Turn to Vision Angle is finished: ", isFinished);
      //return false;
      return timerFinished.get()>0.6;
    }
  }
}
