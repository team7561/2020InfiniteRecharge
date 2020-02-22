package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionController extends SubsystemBase {
	public int ledState;

	public void init()
	{
		turnOnLED();
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
	}
	public void turnOffLED()
	{
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

	}
	public void turnOnLED()
	{
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
	}
	public void blinkLED()
	{
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);

	}
	public double get_tx()
	{
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
	}
	public double get_ty()
	{
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
	}
	public double get_ta()
	{
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
	}
	public boolean has_target()
	{
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false);
	}

}