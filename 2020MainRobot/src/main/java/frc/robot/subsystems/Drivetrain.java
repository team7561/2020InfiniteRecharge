package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import frc.robot.driver.ADIS16448_IMU;

public class Drivetrain extends SubsystemBase {

    double lastError;
    final double encoderRatio = 2;
    public ADXRS450_Gyro gyro;

    CANSparkMax leftA, leftB, leftC, rightA, rightB, rightC;
    public Drivetrain()
    {
        leftA = new CANSparkMax(Ports.DRIVE_LEFT_A_CANID, MotorType.kBrushless);
        leftB = new CANSparkMax(Ports.DRIVE_LEFT_B_CANID, MotorType.kBrushless);
        leftC = new CANSparkMax(Ports.DRIVE_LEFT_C_CANID, MotorType.kBrushless);
        rightA = new CANSparkMax(Ports.DRIVE_RIGHT_A_CANID, MotorType.kBrushless);
        rightB = new CANSparkMax(Ports.DRIVE_RIGHT_B_CANID, MotorType.kBrushless);
        rightC = new CANSparkMax(Ports.DRIVE_RIGHT_C_CANID, MotorType.kBrushless);
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
    }

    //sets the speeds of all driving motors
    public void drive(double leftSpeed, double rightSpeed) {
        leftA.set(leftSpeed);
        leftB.set(leftSpeed);
        leftC.set(leftSpeed);
        rightA.set(-rightSpeed);
        rightB.set(-rightSpeed);
        rightC.set(-rightSpeed);
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
    
    //turns to specified angle
    public void turnToAngle(double m_speed) 
    {

	}
    //put dashboard stuff here
    public void updateDashboard(boolean debug)
    {
        debug = true;
        if (debug)
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
        return (int) (leftA.getEncoder().getPosition()+leftB.getEncoder().getPosition()+leftC.getEncoder().getPosition())/3;
    }
    public int getRightEncoder()
    {
        return (int) -(rightA.getEncoder().getPosition()+rightB.getEncoder().getPosition()+rightC.getEncoder().getPosition())/3;
    }

	public void stop() {
	}

}
