package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.PulseIntakeCommand;
import frc.robot.commands.ShooterFeedCommand;
import frc.robot.commands.intake.IndexCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class WaitAndFeed extends ParallelCommandGroup {

  public WaitAndFeed(FeederSubsystem feeder, IntakeSubsystem intake, ShooterSubsystem shooter) {
    addCommands(
        new ParallelCommandGroup(
            new ShooterFeedCommand(-100, feeder, shooter),
            new IndexCommand(90),
            new PulseIntakeCommand(intake)));
    ;
  }
}
