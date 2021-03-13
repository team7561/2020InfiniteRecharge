package frc.robot.commands.autonomous;

import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ControlPanelManipulator;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.controlpanelmanipulator.*;
import java.nio.file.Path;

    /**
     * Moves the drivetrain along a path for autonomous.
     * Called once if a path is set.
     * Adjusts for deviation using PID.
     * @see RamseteCommand
     */
public class Barrel2 extends SequentialCommandGroup {  
    /**
     * Creates a new DrivePath and processes the given objects to construct the superclass.
     * The superclass handles the rest of the execution.
     * 
     * @param trajectory The selected autonomous code path.
     * @param drivetrain The drivetrain subsystem to be moved in autonomous.
     * @see DrivePath
     */
    Drivetrain m_drivetrain;
    ControlPanelManipulator m_controlPanelManipulator;
    public Barrel2(Drivetrain drivetrain, ControlPanelManipulator controlPanelManipulator) {
        m_drivetrain = drivetrain;
        m_controlPanelManipulator = controlPanelManipulator;
        // Load in path
        String trajectoryJSON = "output/Forward.wpilib.json";
        Trajectory trajectory = new Trajectory();
        try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        
        addCommands ( new DT_SetPose(m_drivetrain, trajectory.getInitialPose()),
                      new CPM_Extend(m_controlPanelManipulator),
                      new DT_DrivePath(trajectory, m_drivetrain),
                      new CPM_Retract(m_controlPanelManipulator));
                      //new DT_Drive_Stop(m_drivetrain));
                    //new CPM_SpinLeft(m_controlPanelManipulator), true);
                    //new CPM_SpinRight(m_controlPanelManipulator), true);
    }
}
