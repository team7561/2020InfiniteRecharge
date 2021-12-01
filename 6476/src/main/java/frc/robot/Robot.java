// Copyright (c) FIRST and other WPILib contributors.  
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Joystick joystick;
  XboxController xbox;

  Timer timer;

  Drive drive;
  Intake intake;
  Shooter shooter;
  ColourWheel colourWheel;
  CameraServer camera1, camera2;
  double servoPosition = 0;
  double servoChange = 0.02;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
   * The device will be automatically initialized with default parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    camera1 = CameraServer.getInstance();
    camera1.startAutomaticCapture(0);
    camera2 = CameraServer.getInstance();
    camera2.startAutomaticCapture(1);

    drive = new Drive();
    intake = new Intake();
    shooter = new Shooter();
    colourWheel = new ColourWheel();

    joystick = new Joystick(RobotMap.joystickPort);
    xbox = new XboxController(RobotMap.xboxPort);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }


  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    if(timer.get()<8)
    {
      shooter.shoot(0.6);
    }
    if(timer.get()>2 && timer.get()<8)
    {
      intake.pickUp();
    }
    if(timer.get() > 8 && timer.get() < 10)
    {
      drive.teleopDrive(0.2, 0);
    }
    if(timer.get() > 8)
    {
      intake.stop();
      shooter.stop();
    }
    if(timer.get() > 10)
    {
      drive.teleopDrive(0,0);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    double value = Math.random();
    System.out.println("Servo Test " + value);
    m_colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes13bit,
        ColorSensorMeasurementRate.kColorRate500ms, GainFactor.kGain6x);

  }

  long prevTime = 0;
  int i = 0;
  long acc = 0;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Drivetrain Controls
    drive.teleopDrive(joystick.getY(), joystick.getX());

    // Intake Controls
    if (joystick.getRawButton(1))
    {
      intake.pickUp();
    }
    else if (joystick.getRawButton(2)) {
      intake.reverse();
    }
    else
    {
      intake.stop();
    }

    // Shooter Controls
    if (xbox.getAButton())
    {
      shooter.shoot(0.6);
    }
    else
    {
      shooter.stop();
    }

    // Colour Wheel Controls
    if (xbox.getXButton())
    {
      colourWheel.forward();
    }
    else if (xbox.getBButton()) {
      colourWheel.reverse();
    }
    else
    {
      colourWheel.stop();
    }

    // Colour Wheel Controls
    if (xbox.getBumper(Hand.kLeft))
    {
      servoPosition -= servoChange;
    }
    else if (xbox.getBumper(Hand.kRight)) {
      servoPosition += servoChange;
    }

    if (servoPosition > 1)
    {
      servoPosition = 1;
    }
    if (servoPosition < -1)
    {
      servoPosition = -1;
    }

    colourWheel.setServo(servoPosition);
    //colourWheel.setServo(0.5*(joystick.getThrottle()+1));

    long timeNow = System.currentTimeMillis();
    long delta = timeNow - prevTime;
    acc += delta;
    if (i > 100) {
      System.out.println(acc / 100.0);
      i = 0;
      acc = 0;
    }
    i += 1;
    prevTime = timeNow;

    Color detectedColor = m_colorSensor.getColor();

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
     */
    System.out.println("Col " + detectedColor);

    System.out.println("Red " + m_colorSensor.getRed());
    System.out.println("Green " + m_colorSensor.getGreen());
    System.out.println("Blue " + m_colorSensor.getBlue());

    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
 }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
