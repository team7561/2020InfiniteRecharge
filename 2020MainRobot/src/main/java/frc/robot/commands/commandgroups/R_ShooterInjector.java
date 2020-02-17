package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.shooter.*;
import frc.robot.subsystems.Injector;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.injector.*;

public class R_ShooterInjector extends ParallelCommandGroup {
  /**
   * Creates a new ComplexAuto.
   *
   * @param drive The drive subsystem this command will run on
   * @param hatch The hatch subsystem this command will run on
   */
  public R_ShooterInjector(Shooter shooter, Injector injector) {
        addCommands(
        // Injector Transfer ball
        new Injector_Transfer_Ball(injector),

        // Shoot At Speed
        new ShootAtSpeed(shooter, -3000)
        );
  }

}
