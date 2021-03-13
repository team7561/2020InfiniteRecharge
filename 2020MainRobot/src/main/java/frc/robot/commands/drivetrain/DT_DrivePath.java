package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;
    /**
     * Moves the drivetrain along a path for autonomous.
     * Called once if a path is set.
     * Adjusts for deviation using PID.
     * @see RamseteCommand
     */
public class DT_DrivePath extends RamseteCommand {  
    /**
     * Creates a new DrivePath and processes the given objects to construct the superclass.
     * The superclass handles the rest of the execution.
     * 
     * @param trajectory The selected autonomous code path.
     * @param drivetrain The drivetrain subsystem to be moved in autonomous.
     * @see DrivePath
     */
    public DT_DrivePath(Trajectory trajectory, Drivetrain drivetrain) {
        /*super(trajectory, drivetrain::getPose, new RamseteController(), drivetrain.getKinematics(),
                (leftSpeed, rightSpeed) -> {
                    drivetrain.driveClosedLoop(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));
                }, drivetrain);*/
        super(trajectory, drivetrain::getPose, new RamseteController(), drivetrain.getKinematics(),
        (leftSpeed, rightSpeed) -> {
            drivetrain.driveClosedLoop(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));
        }, drivetrain);
    }
      
}
