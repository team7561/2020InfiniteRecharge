package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.*;
import frc.robot.commands.TimerCommand;
import frc.robot.commands.drivetrain.TurnToVisionAngle;
import frc.robot.commands.injector.*;
import frc.robot.commands.intakehopper.ExtendHopper;
import frc.robot.commands.intakehopper.RetractHopper;
import frc.robot.commands.intakehopper.ToggleHopper;

public class AutoStrategy1 extends SequentialCommandGroup {
  /**
   * Creates a new ComplexAuto.
   *
   * @param drive The drive subsystem this command will run on
   * @param hatch The hatch subsystem this command will run on
   */
  public AutoStrategy1(Drivetrain drivetrain, IntakeHopper intakeHopper, Shooter shooter, Injector injector, VisionController visionController) {
      // Start spinning up flywheel
      // Turn to vision angle
      // Set hood to higher

      // Move towards target?

      // Spin injector
      // Toggle hopper

        addCommands(
            new ParallelCommandGroup(
                new ShootAtSpeed(shooter, 2500, false),
                new TurnToVisionAngle(drivetrain, visionController, () -> (0.5))
            ).withTimeout(5),
            new SequentialCommandGroup(
                new Injector_Transfer_Ball(injector),
                new TimerCommand(2),
                new ToggleHopper(intakeHopper),
                new TimerCommand(1),
                new ToggleHopper(intakeHopper),
                new TimerCommand(1),
                new ToggleHopper(intakeHopper)
            ).withTimeout(5),
            new ParallelCommandGroup(
                new Shooting_Stop(shooter),
                new Injector_Stop(injector),
                new RetractHopper(intakeHopper)
            ).withTimeout(5));
  
  }

}
