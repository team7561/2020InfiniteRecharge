package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.DriveToPoint;
import frc.robot.commands.intakehopper.Intake_ExtendHopper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeHopper;

public class Auto1 extends SequentialCommandGroup {

    public Auto1(Drivetrain drivetrain, IntakeHopper intakeHopper) {
        addCommands(
        new DriveToPoint(drivetrain,new Coordinate(4, 5), 43),
        new ParallelCommandGroup(new Intake_ExtendHopper(intakeHopper)),
        new DriveToPoint(drivetrain,new Coordinate(4, 5), 43)
        //addSequential(new cmdTurnToHeading(90));
    
        );
    }
}

