package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    VictorSPX intakeMotor;

    public Intake()
    {
        intakeMotor = new VictorSPX(Constants.CAN_ID_INTAKE);
        
        intakeMotor.setNeutralMode(NeutralMode.Coast);

    }
    public void setSpeed(double speed)
    {
        intakeMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }
}
