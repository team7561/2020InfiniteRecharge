package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import frc.robot.driver.ADIS16448_IMU;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class Drivetrain extends SubsystemBase {

    double lastError;
    int current = 40;
    final double encoderRatio = 2;
    public ADXRS450_Gyro gyro;

    private final DifferentialDrive m_drive;
    private final SpeedControllerGroup m_leftMotors;
    private final SpeedControllerGroup m_rightMotors;

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    CANSparkMax leftA, leftB, leftC, rightA, rightB, rightC;
    public Drivetrain()
    {
        leftA = new CANSparkMax(Ports.DRIVE_LEFT_A_CANID, MotorType.kBrushless);
        leftB = new CANSparkMax(Ports.DRIVE_LEFT_B_CANID, MotorType.kBrushless);
        leftC = new CANSparkMax(Ports.DRIVE_LEFT_C_CANID, MotorType.kBrushless);
        rightA = new CANSparkMax(Ports.DRIVE_RIGHT_A_CANID, MotorType.kBrushless);
        rightB = new CANSparkMax(Ports.DRIVE_RIGHT_B_CANID, MotorType.kBrushless);
        rightC = new CANSparkMax(Ports.DRIVE_RIGHT_C_CANID, MotorType.kBrushless);
        m_leftMotors = new SpeedControllerGroup(leftA, leftB, leftC);
        m_rightMotors = new SpeedControllerGroup(rightA, rightB, rightC);
        m_rightMotors.setInverted(true);
        m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        //m_drive.setSafetyEnabled(false);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));


        gyro = new ADXRS450_Gyro();
        leftA.restoreFactoryDefaults();
        leftB.restoreFactoryDefaults();
        leftC.restoreFactoryDefaults();
        rightA.restoreFactoryDefaults();
        rightB.restoreFactoryDefaults();
        rightC.restoreFactoryDefaults();
        gyro.calibrate();
        
        leftA.getEncoder().setPositionConversionFactor(42);
        leftB.getEncoder().setPositionConversionFactor(42);
        leftC.getEncoder().setPositionConversionFactor(42);
        rightA.getEncoder().setPositionConversionFactor(42);
        rightB.getEncoder().setPositionConversionFactor(42);
        rightC.getEncoder().setPositionConversionFactor(42);
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
    }

    //sets the speeds of all driving motors
    public void drive(double leftSpeed, double rightSpeed) {
        m_drive.tankDrive(leftSpeed, -rightSpeed);
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

    // reads gyro (between 0 and 360)
    public double readGyro()
    {
        return (gyro.getAngle() % 360 + 360) % 360;
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
            drive(left, right);
        }
    }
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(-rightVolts);
        m_drive.feed();
      }
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    //put dashboard stuff here
    public void updateDashboard()
    {
        if (Constants.DEBUG_DRIVETRAIN)
        {
            SmartDashboard.putNumber("Gyro Angle", readGyro());
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
