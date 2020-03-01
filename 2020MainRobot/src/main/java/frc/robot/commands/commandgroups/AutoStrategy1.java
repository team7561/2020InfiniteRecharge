package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.*;
import frc.robot.commands.TimerCommand;
import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.commands.drivetrain.TurnToVisionAngle;
import frc.robot.commands.injector.*;
import frc.robot.commands.intakehopper.ExtendHopper;
import frc.robot.commands.intakehopper.RetractHopper;
import frc.robot.commands.intakehopper.ToggleHopper;

public class AutoStrategy1 extends SequentialCommandGroup {

  public AutoStrategy1(Drivetrain drivetrain, IntakeHopper intakeHopper, Shooter shooter, Injector injector, VisionController visionController) {
      // Start spinning up flywheel
      // Turn to vision angle
      // Set hood to higher

      // Move towards target?

      // Spin injector
      // Toggle hopper

        addCommands(
            //new DriveDistance(drivetrain, 0.4, -600),
            //new DriveDistance(drivetrain, 0.4, 600));
            new ParallelCommandGroup(
                new ShootAtSpeedFinish(shooter, 500),
                new TurnToVisionAngle(drivetrain, visionController, () -> (0.5))
                .withTimeout(5)),
            new ParallelCommandGroup(
                new Injector_Transfer_Ball(injector),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new ToggleHopper(intakeHopper),
                    new WaitCommand(0.5),
                    new ToggleHopper(intakeHopper),
                    new WaitCommand(0.5),
                    new ToggleHopper(intakeHopper),
                    new WaitCommand(0.5),
                    new ToggleHopper(intakeHopper),
                    new WaitCommand(0.5),
                    new ToggleHopper(intakeHopper)
            )).withTimeout(5),
            new ParallelCommandGroup(
                new Shooting_Stop(shooter),
                new Injector_Stop(injector),
                new RetractHopper(intakeHopper)
            ).withTimeout(5),
            new DriveDistance(drivetrain, 0.5, 400),
            new DriveDistance(drivetrain, 0.5, -400));
  }
}
