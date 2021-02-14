package frc.robot.commands.autonomous;

import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.drivetrain.*;
import java.nio.file.Path;

    /**
     * Moves the drivetrain along a path for autonomous.
     * Called once if a path is set.
     * Adjusts for deviation using PID.
     * @see RamseteCommand
     */
public class Bounce extends SequentialCommandGroup {  
    /**
     * Creates a new DrivePath and processes the given objects to construct the superclass.
     * The superclass handles the rest of the execution.
     * 
     * @param trajectory The selected autonomous code path.
     * @param drivetrain The drivetrain subsystem to be moved in autonomous.
     * @see DrivePath
     */
    Drivetrain m_drivetrain;
    public Bounce(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        // Load in path
        String trajectoryJSON = "output/Bounce.wpilib.json";
        String trajectory2JSON = "output/Bounce2.wpilib.json";
        String trajectory3JSON = "output/Bounce3.wpilib.json";
        String trajectory4JSON = "output/Bounce4.wpilib.json";
        Trajectory trajectory = new Trajectory();
        Trajectory trajectory2 = new Trajectory();
        Trajectory trajectory3 = new Trajectory();
        Trajectory trajectory4 = new Trajectory();
        try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        trajectory4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        addCommands ( new DT_InitDrivePose(m_drivetrain, 0.752, -3.383),
                      new DT_DrivePath(trajectory, m_drivetrain),
                      new DT_DrivePath(trajectory2, m_drivetrain),
                      new DT_DrivePath(trajectory3, m_drivetrain),
                      new DT_DrivePath(trajectory4, m_drivetrain),
                      new DT_Drive_Stop(m_drivetrain));
    }
      
}
