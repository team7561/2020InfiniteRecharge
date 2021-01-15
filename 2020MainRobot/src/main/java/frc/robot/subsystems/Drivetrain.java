package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class Drivetrain extends SubsystemBase {

    double lastError;
    int current = 40;
    public ADXRS450_Gyro gyro;

    private final DifferentialDrive m_drive;
    private final SpeedControllerGroup m_leftMotors;
    private final SpeedControllerGroup m_rightMotors;
        
    private SimpleMotorFeedforward m_leftFF;
    private SimpleMotorFeedforward m_rightFF;

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;
    private DifferentialDriveKinematics m_kinematics;

    CANSparkMax leftA, leftB, leftC, rightA, rightB, rightC;
    private CANEncoder m_leftEncoder;
    private CANEncoder m_rightEncoder;
    
    private PIDController m_leftController;
    private PIDController m_rightController;

    public Drivetrain()
    {
        leftA = new CANSparkMax(Ports.DRIVE_LEFT_A_CANID, MotorType.kBrushless);
        leftB = new CANSparkMax(Ports.DRIVE_LEFT_B_CANID, MotorType.kBrushless);
        leftC = new CANSparkMax(Ports.DRIVE_LEFT_C_CANID, MotorType.kBrushless);
        rightA = new CANSparkMax(Ports.DRIVE_RIGHT_A_CANID, MotorType.kBrushless);
        rightB = new CANSparkMax(Ports.DRIVE_RIGHT_B_CANID, MotorType.kBrushless);
        rightC = new CANSparkMax(Ports.DRIVE_RIGHT_C_CANID, MotorType.kBrushless);

        leftA.restoreFactoryDefaults();
        leftB.restoreFactoryDefaults();
        leftC.restoreFactoryDefaults();
        rightA.restoreFactoryDefaults();
        rightB.restoreFactoryDefaults();
        rightC.restoreFactoryDefaults();

        m_leftMotors = new SpeedControllerGroup(leftA, leftB, leftC);
        m_rightMotors = new SpeedControllerGroup(rightA, rightB, rightC);
        m_leftEncoder = leftA.getEncoder();
        m_rightEncoder = rightA.getEncoder();
        
        // set up encoder conversion factor
        double conversionFactor = Constants.DRIVE_GEAR_RATIO * 0.3239;

        rightA.setInverted(true);
        m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        m_drive.setSafetyEnabled(false);

        m_leftEncoder.setVelocityConversionFactor(conversionFactor/60);
        m_leftEncoder.setPositionConversionFactor(conversionFactor);

        m_rightEncoder.setVelocityConversionFactor(conversionFactor/60);
        m_rightEncoder.setPositionConversionFactor(conversionFactor);
        
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        m_kinematics = new DifferentialDriveKinematics(Constants.DRIVE_TRACK_WIDTH);

        m_leftController = new PIDController(0.4, 0, 0);
        m_rightController = new PIDController(0.4, 0, 0);
        
        m_leftFF = new SimpleMotorFeedforward(0.1765, 3.3, 0.341);
        m_rightFF = new SimpleMotorFeedforward(0.1835, 3.24, 0.3645);
  
        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
        resetGyro();

        //adis = new ADIS16448_IMU();
        leftA.setIdleMode(IdleMode.kCoast);
        leftB.setIdleMode(IdleMode.kCoast);
        leftC.setIdleMode(IdleMode.kCoast);
        rightA.setIdleMode(IdleMode.kCoast);
        rightB.setIdleMode(IdleMode.kCoast);
        rightC.setIdleMode(IdleMode.kCoast);

        leftA.setSmartCurrentLimit(current);
        leftB.setSmartCurrentLimit(current);
        leftC.setSmartCurrentLimit(current);
        rightA.setSmartCurrentLimit(current);
        rightB.setSmartCurrentLimit(current);
        rightC.setSmartCurrentLimit(current);

        leftB.follow(leftA);
        leftC.follow(leftA);
        rightB.follow(rightA);
        rightC.follow(rightA);
    }

    //sets the speeds of all driving motors
    public void drive(double leftSpeed, double rightSpeed) {
        m_drive.tankDrive(leftSpeed, rightSpeed);
        m_drive.feed();
    }
    public void resetEncoders()
    {
        leftA.getEncoder().setPosition(0);
        leftB.getEncoder().setPosition(0);
        leftC.getEncoder().setPosition(0);
        rightA.getEncoder().setPosition(0);
        rightB.getEncoder().setPosition(0);
        rightC.getEncoder().setPosition(0);
    }

    // resets gyro
    public void resetGyro()
    {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        try
        {
            return Math.IEEEremainder(gyro.getAngle(), 360);
        }
        catch (Exception e)
        {
            return 0;
        }
    }
    /**
     * Gets the drivetrain's kinematic model.
     */
    public DifferentialDriveKinematics getKinematics() {
        return m_kinematics;
    }
    /**
     * Drives the robot using commanded chassis speeds. Call repeatedly.
     */
    public void driveClosedLoop(DifferentialDriveWheelSpeeds speeds) {

        double left = speeds.leftMetersPerSecond;
        double right = speeds.rightMetersPerSecond;

        double leftVoltage = m_leftFF.calculate(left) + m_leftController.calculate(m_leftEncoder.getVelocity(), left);
        double rightVoltage = m_rightFF.calculate(right) + m_rightController.calculate(m_rightEncoder.getVelocity(), right);

        leftA.setVoltage(leftVoltage);
        rightA.setVoltage(rightVoltage);
    }
    public Rotation2d getGyroRotation() {
        return gyro.getRotation2d();//Rotation2d.fromDegrees(-m_gyroFilter.calculate(m_gyro.getAngle()));
      }
    public void periodic() {
        // update the drivetrain's position estimate
        m_odometry.update(getGyroRotation(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    
        if (Constants.DEBUG_DRIVETRAIN)
        {
            SmartDashboard.putNumber("left_enc", m_leftEncoder.getPosition());
            SmartDashboard.putNumber("right_enc", m_rightEncoder.getPosition());
            // publish debug odometry values
            SmartDashboard.putNumber("odometry_x", m_odometry.getPoseMeters().getX());
            SmartDashboard.putNumber("odometry_y", m_odometry.getPoseMeters().getY());
            SmartDashboard.putNumber("odometry_theta", m_odometry.getPoseMeters().getRotation().getDegrees());
        }
    }
    //teleop driving
    public void arcadeDrive(double x, double y, double speed, boolean inverted) {
        //x = x * Math.abs(x) * speed;
        //y = y * Math.abs(y) * speed;
        SmartDashboard.putNumber("Drivespeed", speed);
        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);

        double right = (-y - x)*speed;
        double left = (- (y - x))*speed;
        if (left > 1) {
            left = 1;
        }
        if (right > 1) {
            right = 1;
        }
        if (inverted == true) {
            drive(-left, -right);
        }
        else
        {
            drive(left, -right);
        }
    }
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(-rightVolts);
        m_drive.feed();
      }
    /**
     * Resets the drivetrain's stored pose and encoder values.
     */
    public void resetPose() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);

        m_odometry.resetPosition(new Pose2d(), getGyroRotation());
    }
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, gyro.getRotation2d());
    }
    //put dashboard stuff here
    public void updateDashboard()
    {
        if (Constants.DEBUG_DRIVETRAIN)
        {
            SmartDashboard.putNumber("Gyro Angle", getGyroRotation().getDegrees());
            SmartDashboard.putNumber("Left A Power", leftA.get());
            SmartDashboard.putNumber("Left B Power", leftB.get());
            SmartDashboard.putNumber("Left C Power", leftC.get());
            SmartDashboard.putNumber("Right A Power", rightA.get());
            SmartDashboard.putNumber("Right B Power", rightB.get());
            SmartDashboard.putNumber("Right C Power", rightC.get());
            SmartDashboard.putNumber("Left A Encoder", leftA.getEncoder().getPosition());
            SmartDashboard.putNumber("Left B Encoder", leftB.getEncoder().getPosition());
            SmartDashboard.putNumber("Left C Encoder", leftC.getEncoder().getPosition());
            SmartDashboard.putNumber("Right A Encoder", rightA.getEncoder().getPosition());
            SmartDashboard.putNumber("Right B Encoder", rightB.getEncoder().getPosition());
            SmartDashboard.putNumber("Right C Encoder", rightC.getEncoder().getPosition());
            SmartDashboard.putNumber("Left A Current", leftA.getOutputCurrent());
            SmartDashboard.putNumber("Left B Current", leftB.getOutputCurrent());
            SmartDashboard.putNumber("Left C Current", leftC.getOutputCurrent());
            SmartDashboard.putNumber("Right A Current", rightA.getOutputCurrent());
            SmartDashboard.putNumber("Right B Current", rightB.getOutputCurrent());
            SmartDashboard.putNumber("Right C Current", rightC.getOutputCurrent());
            
        }
    }
    public int getLeftEncoder()
    {
        return (int) (leftA.getEncoder().getPosition()+leftB.getEncoder().getPosition())/2;
    }
    public int getLeftEncoderRate()
    {
        return (int) (leftA.getEncoder().getVelocity()+leftB.getEncoder().getVelocity()+leftC.getEncoder().getVelocity())/3;
    }
    public int getRightEncoder()
    {
        return (int) -(rightA.getEncoder().getPosition()+rightB.getEncoder().getPosition())/2;
    }
    public int getRightEncoderRate()
    {
        return (int) (rightA.getEncoder().getVelocity()+rightB.getEncoder().getVelocity()+rightC.getEncoder().getVelocity())/3;
    }

	public void stop() {
        drive(0, 0);
    }
    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoder());
  }
}