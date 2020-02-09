package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionController extends SubsystemBase {
	public boolean hasTarget = false;
	public double tx = 0;
	public double ty = 0;
	public double ta = 0;
	public int ledState;
	public void update() {
		hasTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false);
		tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
		ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
		ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

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
		return tx;
	}

}
