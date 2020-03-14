/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.climber.Climb_Stop;
import frc.robot.commands.climber.Climber_Reverse;
import frc.robot.commands.climber.LowerHook;
import frc.robot.commands.climber.RaiseHook;
import frc.robot.commands.commandgroups.AutoStrategy1;
import frc.robot.commands.commandgroups.R_ShooterInjector;
import frc.robot.commands.controlpanelmanipulator.CPM_Extend;
import frc.robot.commands.controlpanelmanipulator.CPM_Retract;
import frc.robot.commands.controlpanelmanipulator.CPM_SpinLeft;
import frc.robot.commands.controlpanelmanipulator.CPM_SpinRight;
import frc.robot.commands.controlpanelmanipulator.CPM_Stop;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.Drive_Stop;
import frc.robot.commands.drivetrain.TurnToVisionAngle;
import frc.robot.commands.injector.Injector_Reverse;
import frc.robot.commands.injector.Injector_Reverse_copy;
import frc.robot.commands.injector.Injector_Stop;
import frc.robot.commands.injector.Injector_Transfer_Ball;
import frc.robot.commands.intakehopper.EjectBall;
import frc.robot.commands.intakehopper.ExtendHopper;
import frc.robot.commands.intakehopper.GrabBall;
import frc.robot.commands.intakehopper.Grabbing_Stop;
import frc.robot.commands.intakehopper.RetractHopper;
import frc.robot.commands.intakehopper.ToggleHopper;
import frc.robot.commands.shooter.ShootAtSpeed;
import frc.robot.commands.shooter.Shooter_Extend;
import frc.robot.commands.shooter.Shooter_Retract;
import frc.robot.commands.shooter.Shooting_Stop;
import frc.robot.commands.visioncontroller.VCTurnOffLED;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Injector;

import frc.robot.subsystems.IntakeHopper;
import frc.robot.subsystems.ControlPanelManipulator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  private final Command autoStrategy1 = new AutoStrategy1(m_drivetrain, m_intakeHopper, m_shooter, m_injector, m_visionController);
  public final Command shooter_stop = new Shooting_Stop(m_shooter);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  //HID
  private Joystick joystick = new Joystick(0); //Logitech Extreme 3D Pro Joysick Controller
  private XboxController xboxController = new XboxController(1); //Logitech Gamepad F310 (Xbox Controller)
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand( new ArcadeDrive(m_drivetrain, () -> joystick.getX(), () -> joystick.getY(), () -> joystick.getThrottle()
    ));
    //m_drivetrain.setDefaultCommand( new ArcadeDrive(m_drivetrain, () -> 0, () -> 0));
    //m_shooter.setDefaultCommand( new Shooting_Stop(m_shooter));
    m_climber.setDefaultCommand( new Climb_Stop(m_climber));
    m_intakeHopper.setDefaultCommand( new Grabbing_Stop(m_intakeHopper));
    m_injector.setDefaultCommand( new Injector_Stop(m_injector));
    m_visionController.setDefaultCommand( new VCTurnOffLED(m_visionController));
    m_ControlPanelManipulator.setDefaultCommand( new CPM_Stop(m_ControlPanelManipulator));
    //m_exampleSubsystem.setDefaultCommand( new ExampleCommand(m_exampleSubsystem));
    // Configure the button bindings
    configureButtonBindings();

    m_chooser.addOption("Auto 1", autoStrategy1);
    
    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void configureButtonBindings() {
    System.out.println("Configure button bindings");
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
    //final JoystickButton button_11 = new JoystickButton(joystick, 11);
    final JoystickButton button_12 = new JoystickButton(joystick, 12);

    //final double joystickThrottle = joystick.getThrottle(); //gets the throttle value on the joystick

    //binding buttons to commands for the Joystick Controller
    trigger.whenPressed(new GrabBall(m_intakeHopper), true); //spins intake while held and not interuptable by other driver
    trigger.whenReleased(new Grabbing_Stop(m_intakeHopper), true); //spins intake while held
    thumb.whenPressed(new EjectBall(m_intakeHopper), true);
    thumb.whenReleased(new Grabbing_Stop(m_intakeHopper), true);
    button_3.whenPressed(new TurnToVisionAngle(m_drivetrain, m_visionController, () -> (joystick.getThrottle()+1)/2).withTimeout(5), true);
    //button_3.whenPressed(new RaiseHook(m_climber), true);
    //button_3.whenReleased(new Climb_Stop(m_climber), true);
    //button_4.whenPressed(new Climb(m_climber), true);
    button_5.whenPressed(new ShootAtSpeed(m_shooter, 3000), true);  // Extend intake
    button_6.whenPressed(new Shooting_Stop(m_shooter), true); // retract inatke
    
    button_7.whenPressed(new ExtendHopper(m_intakeHopper), true); // Shoot at speed
    button_8.whenPressed(new RetractHopper(m_intakeHopper), true);  


    /*button_9.whenPressed(new Climb(m_climber), true); // Stop grabbing
    button_9.whenReleased(new Climb_Stop(m_climber), true);
    button_10.whenPressed(new RaiseHook(m_climber), true);  
    button_10.whenReleased(new Climb_Stop(m_climber), true);
    //button_12.whenReleased(new Drive_Stop(m_drivetrain));
    //button_11.whenPressed(new CPM_Spin(m_ControlPanelManipulator), true);
    //button_11.whenReleased(new CPM_Stop(m_ControlPanelManipulator), true);
    //button_12.whenPressed(new Injector_Transfer_Ball(m_injector), true);
    button_12.whenPressed(new Climber_Reverse(m_climber), true);
    button_12.whenReleased(new Climb_Stop(m_climber), true);*/
    
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

    final JoystickAnalogButton LT = new JoystickAnalogButton(xboxController, 5);
    //final JoystickAnalogButton RT = new JoystickAnalogButton(xboxController, 6);
  
    final DPadButton dpad_Up = new DPadButton(xboxController, DPadButton.Direction.UP);
    final DPadButton dpad_Down = new DPadButton(xboxController, DPadButton.Direction.DOWN);
    final DPadButton dpad_Left = new DPadButton(xboxController, DPadButton.Direction.LEFT);
    final DPadButton dpad_Right = new DPadButton(xboxController, DPadButton.Direction.RIGHT);
    //binding buttons to commands for the Xbox Controller
    
    button_A.whenPressed(new Injector_Transfer_Ball(m_injector), true);
    button_A.whenReleased(new Injector_Stop(m_injector), true);
    button_B.whenPressed(new Injector_Reverse(m_injector), true);
    button_B.whenReleased(new Injector_Stop(m_injector), true);
    button_X.whenPressed(new Shooter_Extend(m_shooter), false);
    button_Y.whenPressed(new Shooter_Retract(m_shooter), true);
    //button_LB.whenPressed(new R_ShooterInjector(m_shooter, m_injector), true);
    button_RB.whenPressed(new ToggleHopper(m_intakeHopper), true);
    //button_RB.whenPressed(new VCTurnOffLED(m_visionController), true);

    //back.whenPressed(new CPM_Extend(m_ControlPanelManipulator), true);
    //start.whenPressed(new CPM_Retract(m_ControlPanelManipulator), true);

    left_joystick_button.whenPressed(new CPM_Stop(m_ControlPanelManipulator), true);
    right_joystick_button.whenPressed(new Climb_Stop(m_climber), true);

    //RT.whenPressed(new Shooting_Stop(m_shooter), true);
    //LT.whenPressed(new Shooting_Stop(m_shooter), true);

    dpad_Up.whenPressed(new CPM_Extend(m_ControlPanelManipulator), true);
    dpad_Down.whenPressed(new CPM_Retract(m_ControlPanelManipulator), true);
    dpad_Left.whenPressed(new CPM_SpinLeft(m_ControlPanelManipulator), true);
    dpad_Right.whenPressed(new CPM_SpinRight(m_ControlPanelManipulator), true);

    }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
        System.out.println("Command is " + autoStrategy1.toString());
        return autoStrategy1;
        //return m_chooser.getSelected();
    // Create a voltage constraint to ensure we don't accelerate too fast
    /*var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_drivetrain::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter,
        Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrain::tankDriveVolts,
        m_drivetrain
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));*/
  }
  
}
