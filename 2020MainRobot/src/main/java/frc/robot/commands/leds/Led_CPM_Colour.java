/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.leds;

import frc.robot.subsystems.LEDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class Led_CPM_Colour extends CommandBase {
  private final LEDController m_subsystem;
  private String m_colour;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Led_CPM_Colour(LEDController subsystem, String colour) {
    m_subsystem = subsystem;
    m_colour = colour;
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
    if (m_colour == "Blue"){
      m_subsystem.Blue();
    }
    if (m_colour == "Red"){
      m_subsystem.Red();
    }
    if (m_colour == "Green"){
      m_subsystem.Green();
    }
    if (m_colour == "Yellow"){
      m_subsystem.Yellow();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
