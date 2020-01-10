/*----------------------------------------------------------------------------*/                                            
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */                                            
/* Open Source Software - may be modified and shared by FRC teams. The code   */                                            
/* must be accompanied by the FIRST BSD license file in the root directory of */                                            
/* the project.                                                               */                                            
/*----------------------------------------------------------------------------*/                                            

package frc.robot;                                                                                    

import edu.wpi.first.wpilibj.*;                                                                                    
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;                                                                                    
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;                                                                                    
import frc.robot.subsystems.Climber;                                                                                    
import frc.robot.subsystems.Drivetrain;                                                                                    
/*                                                                                   
import edu.wpi.first.wpilibj.networktables.NetworkTable;*/                                                                                    
import edu.wpi.first.cameraserver.CameraServer;                                                                                    
import edu.wpi.first.networktables.NetworkTable;                                                                                    
import edu.wpi.first.wpilibj.Timer;                                                                                    
import edu.wpi.first.wpilibj.PowerDistributionPanel;                                                                                    
import frc.robot.subsystems.ColourSensor;

public class Robot extends TimedRobot {                                                                                    
  private static final String kDefaultAuto = "Default";                                                                                    
  private static final String kCustomAuto = "My Auto";                                                                                    
  private String m_autoSelected;                                                                                    
  double curr_angle, target_angle;                                                                                    
  public Joystick joystick = new Joystick(1);                                                                                    
  public XboxController xboxController = new XboxController(2);                                                                                    
  public Climber climber = new Climber();                                                                                    
  public PowerCellIntake PowerCellIntake = new PowerCellIntake();                                                                                    
  public Drivetrain drivetrain = new Drivetrain();                                                                                    
  public VisionController visionController = new VisionController();                                            
  public LEDController ledController = new LEDController();   
  public ColourSensor colourSensor = new ColourSensor();                                         
  Timer matchTimer = new Timer();                                            
  NetworkTable table;                                            
  String autoMode;                                            

  public PowerDistributionPanel pdp;                                            
  boolean invertedDrive;                                            
  double speedControl;                                            
  boolean debug;                                            
  private final SendableChooser<String> m_chooser = new SendableChooser<>();                                            


  @Override                                            
  public void robotInit() {                                            
    debug = true;                                            
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);                                            
    invertedDrive = false;                                            
    speedControl = 0.5;                                            

   /* table = NetworkTable.getTable("GRIP/myContoursReport");*/                                            
    CameraServer.getInstance().startAutomaticCapture();                                            
  }                                            

  @Override                                            
  public void robotPeriodic() {                                            
  }                                                 
    
 /* @Override                                                 
  public void autonomousInit() {                                                                                            
    matchTimer.start();                                                                                            
    drivetrain.resetEncoders();                                                                                            
    autoMode = m_chooser.getSelected();                                                                                            
    
    if (autoMode == "Pathfinder")                                                                                            
    {                                                                                            
      Object pathweaver;                                                                                            
      pathweaver.wait(drivetrain);                                                                                            
    }                                                                                            
    else                                                                                            
    {                                                                                            
    
    }                                                                                            
    strategy.reset();                                                                                            
  }                                                                                                                                              
                                                                                                  
                                                                                                  
  @Override                                                                                                                                         
  public void autonomousPeriodic() {                                                                                                                                         
    //pathweaver.followPath(drivetrain);                                                                                                                                         
    strategy.run(this);                                                                                                                                         
    updateDashboards();                                                                                                                                         
  }                                                                                                                                         
                                                                                              
  @Override                                                                                                                                         
  public void teleopInit() {                                                                                                                                         
    climber.recoverCarrige();                                                                                                                                         
    panelintake.getPannel();                                                                                                                                         
    climber.stopVacuum();                                                                                                                                         
    drivetrain.resetEncoders();                                                                                                                                         
  }                                                                                                                                         
   */                                                                                                                                     
  @Override                                                                                                                                         
  public void teleopPeriodic() {                                                                                                                
                                                                                                                                       
    updateDashboards();                                                                                                                                        
  }                                                                                                                                                                                                        
  private void drive() {                                                                                                                                         
  }                                                                                                                                         
                                                                                              
  @Override                                                                                                                                         
  public void testPeriodic() {                                                                                                                                         
  }                                                                                                                                         
  public void updateDashboards()                                                                                                                                        
  {                                                                                                                                         
    PowerCellIntake.updateDashboard(true);                                                                                                                                         
    //climber.updateDashboard(debug);                                                                                                                                         
    //drivetrain.updateDashboard(debug);                                                                                                                                         
    //visionController.updateDashboard(debug);                                                                                                                                         
    //panelintake.updateDashboard(debug);
    colourSensor.robotPeriodic();
    colourSensor.updateDashboard(true);                                                                                                                                         
  }                                           
}                                           
