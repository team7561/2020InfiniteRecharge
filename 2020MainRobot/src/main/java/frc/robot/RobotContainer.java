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
import frc.robot.commands.climber.Climb;
import frc.robot.commands.climber.Climb_Stop;
import frc.robot.commands.climber.LowerHook;
import frc.robot.commands.climber.RaiseHook;
import frc.robot.commands.controlpanelmanipulator.CPM_Extend;
import frc.robot.commands.controlpanelmanipulator.CPM_Retract;
import frc.robot.commands.controlpanelmanipulator.CPM_Spin;
import frc.robot.commands.controlpanelmanipulator.CPM_Stop;
import frc.robot.commands.controlpanelmanipulator.SpinPositionControl;
import frc.robot.commands.controlpanelmanipulator.SpinToColour;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.TurnToVisionAngle;
import frc.robot.commands.injector.Injector_Stop;
import frc.robot.commands.injector.Injector_Transfer_Ball;
import frc.robot.commands.IntakeHopper.ExtendHopper;
import frc.robot.commands.IntakeHopper.GrabBall;
import frc.robot.commands.IntakeHopper.Grabbing_Stop;
import frc.robot.commands.IntakeHopper.RetractHopper;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAtSpeed;
import frc.robot.commands.shooter.Shooting_Stop;
import frc.robot.commands.visioncontroller.VCBlink_LED;
import frc.robot.commands.visioncontroller.VCTurnOffLED;
import frc.robot.commands.visioncontroller.VCTurnOnLED;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Injector;
import frc.robot.subsystems.IntakeHopper;
import frc.robot.subsystems.ControlPanelManipulator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionController;
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
  private final Climber m_climber = new Climber();
  private final IntakeHopper m_intakeHopper = new IntakeHopper();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Shooter m_shooter = new Shooter();
  private final Injector m_injector = new Injector();
  private final VisionController m_visionController = new VisionController();
  private final ControlPanelManipulator m_ControlPanelManipulator = new ControlPanelManipulator();

  //HID
  private Joystick joystick = new Joystick(0); //Logitech Extreme 3D Pro Joysick Controller
  private XboxController xboxController = new XboxController(1); //Logitech Gamepad F310 (Xbox Controller)
  
  
 // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
 
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand( new ArcadeDrive(m_drivetrain, () -> joystick.getX(), () -> joystick.getY()));
    //m_drivetrain.setDefaultCommand( new ArcadeDrive(m_drivetrain, () -> 0, () -> 0));
    m_shooter.setDefaultCommand( new Shooting_Stop(m_shooter));
    m_climber.setDefaultCommand( new Climb_Stop(m_climber));
    m_intakeHopper.setDefaultCommand( new Grabbing_Stop(m_intakeHopper));
    m_injector.setDefaultCommand( new Injector_Stop(m_injector));
    m_visionController.setDefaultCommand( new VCTurnOnLED(m_visionController));
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

    //final double joystickThrottle = joystick.getThrottle(); //gets the throttle value on the joystick

    //binding buttons to commands for the Joystick Controller
    trigger.whileHeld(new GrabBall(m_intakeHopper), true); //spins intake while held
    thumb.whenPressed(new Shooting_Stop(m_shooter), true); //Lock Drivetrain????

    button_3.whenPressed(new RaiseHook(m_climber), true);
    button_3.whenReleased(new Climb_Stop(m_climber), true);          // Stop Climbing
    button_4.whenPressed(new Climb(m_climber), true);
    
    button_5.whenPressed(new ExtendHopper(m_intakeHopper), true);  // Extend intake
    button_6.whenPressed(new RetractHopper(m_intakeHopper), true); // retract inatke
    
    button_7.whenPressed(new ShootAtSpeed(m_shooter, 300), true);  // Shoot at speed
    button_8.whenPressed(new LowerHook(m_climber), true);          // Lower hook
    button_8.whenReleased(new Climb_Stop(m_climber), true);          // Stop Climbing

    button_9.whenPressed(new Grabbing_Stop(m_intakeHopper), true); // Stop grabbing
    button_10.whileHeld(new Shoot(m_shooter), true);               // Shoot

    button_11.whenPressed(new CPM_Spin(m_ControlPanelManipulator), true);
    button_11.whenReleased(new CPM_Stop(m_ControlPanelManipulator), true);
    button_12.whenPressed(new Injector_Transfer_Ball(m_injector), true);
    button_12.whenReleased(new Injector_Stop(m_injector), true);
    
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


    //final double RT = xboxController.getTriggerAxis(Hand.kRight); //probably the value of the right trigger
    //final double LT = xboxController.getTriggerAxis(Hand.kLeft); //probably the value of the left trigger

    //binding buttons to commands for the Xbox Controller
    
    button_A.whenPressed(new LowerHook(m_climber), true);
    button_B.whenPressed(new Climb(m_climber), true);
    button_X.whileHeld(new TurnToVisionAngle(m_drivetrain, m_visionController, 0.4), false);
    button_Y.whenPressed(new RaiseHook(m_climber), true);
    
    back.whenPressed(new VCBlink_LED(m_visionController), true);
    start.whenPressed(new VCTurnOffLED(m_visionController), true);

    //back.whenPressed(new CPM_Extend(m_ControlPanelManipulator), true);
    //start.whenPressed(new CPM_Retract(m_ControlPanelManipulator), true);

    left_joystick_button.whenPressed(new CPM_Stop(m_ControlPanelManipulator), true);
    right_joystick_button.whenPressed(new Climb_Stop(m_climber), true);

    button_8.whenPressed(new SpinPositionControl(m_ControlPanelManipulator), true);
    button_9.whenActive(new SpinToColour(m_ControlPanelManipulator, "Red"), true);
    button_10.whenActive(new SpinToColour(m_ControlPanelManipulator, "Green"), true);
    button_11.whenActive(new SpinToColour(m_ControlPanelManipulator, "Yellow"), true);
    button_12.whenActive(new SpinToColour(m_ControlPanelManipulator, "Blue"), true);

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
