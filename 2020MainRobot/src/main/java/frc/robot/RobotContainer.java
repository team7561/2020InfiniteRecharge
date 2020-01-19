/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.climber.Climb_Stop;
import frc.robot.commands.controlpanelmanipulator.CPM_Stop;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.intakehopper.Grabbing_Stop;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.Shooting_Stop;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeHopper;
import frc.robot.subsystems.ControlPanelManipulator;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Climber m_climber = new Climber();
  private final IntakeHopper m_iIntakeHopper = new IntakeHopper();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Shooter m_shooter = new Shooter();
  private final ControlPanelManipulator m_ControlPanelManipulator = new ControlPanelManipulator();
  private Joystick joystick = new Joystick(0);
  
 // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
 
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand( new ArcadeDrive(m_drivetrain, () -> joystick.getX(), () -> joystick.getY()));
    m_shooter.setDefaultCommand( new Shooting_Stop(m_shooter));
    m_climber.setDefaultCommand( new Climb_Stop(m_climber));
    m_iIntakeHopper.setDefaultCommand( new Grabbing_Stop(m_iIntakeHopper));
    m_ControlPanelManipulator.setDefaultCommand( new CPM_Stop(m_ControlPanelManipulator));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    System.out.println("Configure button bindings");
    final JoystickButton trigger = new JoystickButton(joystick, 1);

    trigger.whileHeld(new Shoot(m_shooter), true);

    }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
