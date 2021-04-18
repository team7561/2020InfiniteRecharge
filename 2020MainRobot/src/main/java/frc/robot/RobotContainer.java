/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.trajectory.constraint.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.climber.*;
import frc.robot.commands.controlpanelmanipulator.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.injector.*;
import frc.robot.commands.intakehopper.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.visioncontroller.*;
import frc.robot.autonomous.Auto1;
import frc.robot.commands.LED_Controller.LED_Select_Random_Colour;
import frc.robot.commands.autonomous.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final Climber m_climber = new Climber();
  private final IntakeHopper m_intakeHopper = new IntakeHopper();
  private final Drivetrain m_drivetrain = new Drivetrain();
  public final Shooter m_shooter = new Shooter();
  private final Injector m_injector = new Injector();
  private final VisionController m_visionController = new VisionController();
  private final LEDController m_ledcontroller = new LEDController();
  private final ControlPanelManipulator m_ControlPanelManipulator = new ControlPanelManipulator();

  //HID
  private Joystick joystick = new Joystick(0); //Logitech Extreme 3D Pro Joysick Controller
  private XboxController xboxController = new XboxController(1); //Logitech Gamepad F310 (Xbox Controller)
  
  TrajectoryConfig m_trajConfig = new TrajectoryConfig(Constants.AUTO_MAX_VELOCITY, Constants.AUTO_MAX_ACCEL);

  SendableChooser<List<Pose2d>> m_autoChooser = new SendableChooser<List<Pose2d>>();

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    m_drivetrain.setDefaultCommand( new DT_ArcadeDrive(m_drivetrain, () -> joystick.getX(), () -> joystick.getY(), () -> joystick.getThrottle()
    ));
    //m_drivetrain.setDefaultCommand( new ArcadeDrive(m_drivetrain, () -> 0, () -> 0));
    m_shooter.setDefaultCommand( new Shooter_Shooting_Stop(m_shooter));
    m_climber.setDefaultCommand( new Climb_StopWinch(m_climber));
    m_intakeHopper.setDefaultCommand( new Intake_Grabbing_Stop(m_intakeHopper));
    m_injector.setDefaultCommand( new Injector_Stop(m_injector));
    m_visionController.setDefaultCommand( new VC_TurnOnLED(m_visionController));
    m_ControlPanelManipulator.setDefaultCommand( new CPM_Stop(m_ControlPanelManipulator));
    
    // Configure the button bindings
    configureButtonBindings();

   // add trajectory constraints
   m_trajConfig.addConstraint(new DifferentialDriveKinematicsConstraint(m_drivetrain.getKinematics(), 1.6));
   m_trajConfig.addConstraint(new CentripetalAccelerationConstraint(Constants.AUTO_MAX_CENTRIPETAL_ACCEL));
   m_trajConfig.addConstraint(new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,Constants.kvVoltSecondsPerMeter,Constants.kaVoltSecondsSquaredPerMeter),m_drivetrain.getKinematics(),2));


    // set up autonomous trajectories
     m_chooser.setDefaultOption("None", null);
     m_chooser.addOption("Barrel", new Barrel2(m_drivetrain, m_ControlPanelManipulator)); 
     m_chooser.addOption("Bounce", new Bounce(m_drivetrain)); 
     m_chooser.addOption("Slalom", new Slalom(m_drivetrain)); 
     m_chooser.addOption("PathABlue", new PathABlue(m_drivetrain)); 
     m_chooser.addOption("PathARed", new PathBRed(m_drivetrain)); 

     SmartDashboard.putData(m_chooser);
    
    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }
  
  private void configureButtonBindings() {
    System.out.println("Configure button bindings");
    //creating the buttons for the Joystick Controller

    SmartDashboard.putData("Reset Hood Position", new Shooter_Reset_Hood(m_shooter));
    SmartDashboard.putData("Hood Auto", new Shooter_Auto_Hood(m_shooter, true));
    SmartDashboard.putData("Hood Manual", new Shooter_Auto_Hood(m_shooter, false));
    SmartDashboard.putData("Hood Close", new Shooter_Set_Hood_Setpoint(m_shooter, 0));
    SmartDashboard.putData("Hood Far", new Shooter_Set_Hood_Setpoint(m_shooter, 2));
    SmartDashboard.putData("Hood Vision True", new Shooter_Auto_Vision_Speed(m_shooter, true));
    SmartDashboard.putData("Hood Vision False", new Shooter_Auto_Vision_Speed(m_shooter, false));

    final JoystickButton trigger =   new JoystickButton(joystick, 1);
    final JoystickButton thumb   =   new JoystickButton(joystick, 2);
    final JoystickButton button_3 =  new JoystickButton(joystick, 3);
    final JoystickButton button_4 =  new JoystickButton(joystick, 4);
    final JoystickButton button_5 =  new JoystickButton(joystick, 5);
    final JoystickButton button_6 =  new JoystickButton(joystick, 6);
    final JoystickButton button_7 =  new JoystickButton(joystick, 7);
    final JoystickButton button_8 =  new JoystickButton(joystick, 8);
    final JoystickButton button_9 =  new JoystickButton(joystick, 9);
    final JoystickButton button_10 = new JoystickButton(joystick, 10);
    final JoystickButton button_11 = new JoystickButton(joystick, 11);
    final JoystickButton button_12 = new JoystickButton(joystick, 12);

    //binding buttons to commands for the Joystick Controller
    trigger.whenPressed(new Intake_GrabBall(m_intakeHopper), true); //spins intake while held and not interuptable by other driver
    trigger.whenReleased(new Intake_Grabbing_Stop(m_intakeHopper), true); //spins intake while held
    thumb.whenPressed(new Intake_EjectBall(m_intakeHopper), true); // Eject the ball
    thumb.whenReleased(new Intake_Grabbing_Stop(m_intakeHopper), true); // Stop grabbing the ball
    button_3.whenPressed(new DT_TurnToVisionAngle(m_drivetrain, m_visionController, () -> (joystick.getThrottle()+1)/2).withTimeout(5), true); // Turns the drivetrain to the right vision angle
    button_4.whenPressed(new DT_ResetDrivePose(m_drivetrain), true); // Start Winch
    //button_4.whenPressed(new Climb_StartWinch(m_climber), true); // Start Winch
    button_5.whenPressed(new Shooter_ShootAtSpeed(m_shooter), true);  // Shoot at speed
    button_6.whenPressed(new Shooter_Shooting_Stop(m_shooter), true); // Stop shooting
    
    button_7.whenPressed(new Intake_RetractHopper(m_intakeHopper), true); // Extend Hopper
    button_8.whenPressed(new Intake_ExtendHopper(m_intakeHopper), true); // Reatract Hopper

    button_9.whenPressed(new Climb_StartWinch(m_climber), true); // Stop grabbing
    button_9.whenReleased(new Climb_StopWinch(m_climber), true); // Stop the Winch
    button_10.whenPressed(new Climb_RaiseHook(m_climber), true); // Raise the Hook
    button_10.whenReleased(new Climb_StopWinch(m_climber), true); // Stop the Winch
    button_11.whenPressed(new Shooter_Retract(m_shooter), true);  // Extend the Shooter Hood
    button_11.whenReleased(new Shooter_Stop_Hood(m_shooter), true); // Stop the Shooter Hood
    button_12.whenPressed(new Shooter_Extend(m_shooter), true); // Retract the Shooter Hood
    button_12.whenReleased(new Shooter_Stop_Hood(m_shooter), true); // Stop the Shooter Hood
    
    //creating the buttons for the Xbox Controller
    final JoystickButton button_A = new JoystickButton(xboxController, 1);
    final JoystickButton button_B = new JoystickButton(xboxController, 2);
    final JoystickButton button_X = new JoystickButton(xboxController, 3);
    final JoystickButton button_Y = new JoystickButton(xboxController, 4);

    //final JoystickButton button_LB = new JoystickButton(xboxController, 5);
    final JoystickButton button_RB = new JoystickButton(xboxController, 6);

    //final JoystickButton back = new JoystickButton(xboxController, 7);
    final JoystickButton start = new JoystickButton(xboxController, 8);
    //final JoystickButton left_joooooooooooooystick_button = new JoystickButton(xboxController, 9);
    //final JoystickButton right_joystick_button = new JoystickButton(xboxController, 10);
    //final JoystickAnalogButton LT = new JoystickAnalogButton(xboxController, 2);
    final JoystickAnalogButton RT = new JoystickAnalogButton(xboxController, 3);
  
    final DPadButton dpad_Up = new DPadButton(xboxController, DPadButton.Direction.UP);
    final DPadButton dpad_Down = new DPadButton(xboxController, DPadButton.Direction.DOWN);
    final DPadButton dpad_Left = new DPadButton(xboxController, DPadButton.Direction.LEFT);
    final DPadButton dpad_Right = new DPadButton(xboxController, DPadButton.Direction.RIGHT);
    //binding buttons to commands for the Xbox Controller
    
    button_A.whenPressed(new Injector_Transfer_Ball(m_injector), true);
    button_A.whenReleased(new Injector_Stop(m_injector), true);
    button_B.whenPressed(new Injector_Reverse(m_injector), true);
    button_B.whenReleased(new Injector_Stop(m_injector), true);
    button_X.whenPressed(new Shooter_Retract(m_shooter), true);
    button_X.whenReleased(new Shooter_Stop_Hood(m_shooter), true); // Stop the Shooter Hood
    button_Y.whenPressed(new Shooter_Extend(m_shooter), true);
    button_Y.whenReleased(new Shooter_Stop_Hood(m_shooter), true); // Stop the Shooter Hood
    //button_LB.whenPressed(new R_ShooterInjector(m_shooter, m_injector), true);
    start.whenPressed(new LED_Select_Random_Colour(m_ledcontroller), true);
    button_RB.whenPressed(new Intake_ToggleHopper(m_intakeHopper), true);

    RT.whenPressed(new Climb_ReverseWinch(m_climber), true);
    RT.whenReleased(new Climb_StopWinch(m_climber), true);

    dpad_Up.whenPressed(new CPM_Extend(m_ControlPanelManipulator), true);
    dpad_Down.whenPressed(new CPM_Retract(m_ControlPanelManipulator), true);
    dpad_Left.whenPressed(new CPM_SpinLeft(m_ControlPanelManipulator), true);
    dpad_Right.whenPressed(new CPM_SpinRight(m_ControlPanelManipulator), true);
  }
  
  public Command getAutonomousCommand3() {

    List<Pose2d> waypoints = m_autoChooser.getSelected();

    if (waypoints != null) {

      Trajectory traj = TrajectoryGenerator.generateTrajectory(waypoints, m_trajConfig);

      return new DT_ResetDrivePose(m_drivetrain).andThen(new DT_DrivePath(traj, m_drivetrain)).andThen(() -> {
        m_drivetrain.stop();
      });

    } else {
      return null;
    }
  }

  public Command getAutonomousCommand() {
    return new Auto1(m_drivetrain, m_intakeHopper, m_shooter, m_injector, m_ledcontroller, m_visionController);
  }
  /*
  TrajectoryConfig config = new TrajectoryConfig(0.1, 0.1);
  config.setKinematics(m_drivetrain.getKinematics());
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
      config
  );
  RamseteCommand command = new RamseteCommand(
    trajectory,
    m_drivetrain::getPose,
    new RamseteController(2, .7),
    m_drivetrain.getFeedforward(),
    m_drivetrain.getKinematics(),
    m_drivetrain::getWheelSpeeds,
    m_drivetrain.getLeftPIDController(),
    m_drivetrain.getRightPIDController(),
    m_drivetrain::setOutputVolts,
    m_drivetrain
  );*/
  //return new DT_InitDrivePose(m_drivetrain, 0,0).andThen(command.andThen(() -> m_drivetrain.setOutputVolts(0, 0)));

 // return m_chooser.getSelected();
  //}  
}