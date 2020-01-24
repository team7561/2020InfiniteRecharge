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
import frc.robot.commands.climber.Climb;
import frc.robot.commands.climber.Climb_Stop;
import frc.robot.commands.climber.LowerHook;
import frc.robot.commands.climber.RaiseHook;
import frc.robot.commands.controlpanelmanipulator.CPM_Extend;
import frc.robot.commands.controlpanelmanipulator.CPM_Retract;
import frc.robot.commands.controlpanelmanipulator.CPM_Stop;
import frc.robot.commands.controlpanelmanipulator.SpinPositionControl;
import frc.robot.commands.controlpanelmanipulator.SpinToColour;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.IntakeHopper.ExtendHopper;
import frc.robot.commands.IntakeHopper.GrabBall;
import frc.robot.commands.IntakeHopper.Grabbing_Stop;
import frc.robot.commands.IntakeHopper.RetractHopper;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAtSpeed;
import frc.robot.commands.shooter.Shooting_Stop;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeHopper;
import frc.robot.subsystems.LEDController;
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
  private final IntakeHopper m_intakeHopper = new IntakeHopper();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Shooter m_shooter = new Shooter();
  private final ControlPanelManipulator m_ControlPanelManipulator = new ControlPanelManipulator();

  private final LEDController m_ledcontroller = new LEDController();

  //HID
  private Joystick joystick = new Joystick(0); //Logitech Extreme 3D Pro Joysick Controller
  private XboxController xboxController = new XboxController(1); //Logitech Gamepad F310 (Xbox Controller)
  
 // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
 
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand( new ArcadeDrive(m_drivetrain, () -> joystick.getX(), () -> joystick.getY()));
    m_drivetrain.setDefaultCommand( new ArcadeDrive(m_drivetrain, () -> 0, () -> 0));
    m_shooter.setDefaultCommand( new Shooting_Stop(m_shooter));
    m_climber.setDefaultCommand( new Climb_Stop(m_climber));
    m_intakeHopper.setDefaultCommand( new Grabbing_Stop(m_intakeHopper));
    m_ControlPanelManipulator.setDefaultCommand( new CPM_Stop(m_ControlPanelManipulator));
    //m_exampleSubsystem.setDefaultCommand( new ExampleCommand(m_exampleSubsystem));
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

    //example create joystick: final JoystickButton name = new JoystickButton(joystick, controller button ID)
    //example create binding: name.command(new exampleCommand())

    //creating the buttons for the Joystick Controller
    final JoystickButton trigger = new JoystickButton(joystick, 1);
    final JoystickButton thumb = new JoystickButton(joystick, 2);

    final JoystickButton button_3 = new JoystickButton(joystick, 3);
    final JoystickButton button_4 = new JoystickButton(joystick, 4);

    final JoystickButton button_5 = new JoystickButton(joystick, 5);
    final JoystickButton button_6 = new JoystickButton(joystick, 6);

    final JoystickButton button_7 = new JoystickButton(joystick, 7);
    final JoystickButton button_8 = new JoystickButton(joystick, 8);

    final JoystickButton button_9 = new JoystickButton(joystick, 9);
    final JoystickButton button_10 = new JoystickButton(joystick, 10);

    final JoystickButton button_11 = new JoystickButton(joystick, 11);
    final JoystickButton button_12 = new JoystickButton(joystick, 12);

    //binding buttons to commands for the Joystick Controller
    trigger.whileHeld(new Shoot(m_shooter), true);
    thumb.whenPressed(new Shooting_Stop(m_shooter), true);

    button_3.whenPressed(new ShootAtSpeed(m_shooter, -500), true);
    button_4.whenPressed(new GrabBall(m_intakeHopper), true);
    /*
    button_5.whenPressed(new ExtendHopper(m_intakeHopper), true);
    button_6.whenPressed(new RetractHopper(m_intakeHopper), true);

    button_7.whenPressed(new ExampleCommand(m_exampleSubsystem), true);
    button_8.whenPressed(new ExampleCommand(m_exampleSubsystem), true);

    button_9.whenPressed(new ExampleCommand(m_exampleSubsystem), true);
    button_10.whenPressed(new ExampleCommand(m_exampleSubsystem), true);

    button_11.whenPressed(new ExampleCommand(m_exampleSubsystem), true);
    button_12.whenPressed(new ExampleCommand(m_exampleSubsystem), true);
    */
    //creating the buttons for the Xbox Controller
    final JoystickButton button_A = new JoystickButton(xboxController, 1);
    final JoystickButton button_B = new JoystickButton(xboxController, 2);
    final JoystickButton button_X = new JoystickButton(xboxController, 3);
    final JoystickButton button_Y = new JoystickButton(xboxController, 4);

    final JoystickButton button_LB = new JoystickButton(xboxController, 5);
    final JoystickButton button_RB = new JoystickButton(xboxController, 6);

    final JoystickButton back = new JoystickButton(xboxController, 7);
    final JoystickButton start = new JoystickButton(xboxController, 8);

    final JoystickButton left_joystick_button = new JoystickButton(xboxController, 9);
    final JoystickButton right_joystick_button = new JoystickButton(xboxController, 10);

    //binding buttons to commands for the Xbox Controller
    
    button_A.whenPressed(new LowerHook(m_climber), true);
    button_B.whenPressed(new Climb(m_climber), true);
    button_X.whenPressed(new ExampleCommand(m_exampleSubsystem), true);
    button_Y.whenPressed(new RaiseHook(m_climber), true);

    button_LB.whenPressed(new SpinPositionControl(m_ControlPanelManipulator), true);
    button_RB.whenPressed(new SpinToColour(m_ControlPanelManipulator, null), true);

    back.whenPressed(new CPM_Extend(m_ControlPanelManipulator), true);
    start.whenPressed(new CPM_Retract(m_ControlPanelManipulator), true);

    left_joystick_button.whenPressed(new CPM_Stop(m_ControlPanelManipulator), true);
    right_joystick_button.whenPressed(new Climb_Stop(m_climber), true);

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
