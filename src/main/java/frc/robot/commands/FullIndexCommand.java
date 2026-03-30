package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.IndexCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FullIndexCommand extends ParallelCommandGroup {

  public FullIndexCommand(
      FeederSubsystem feeder, IntakeSubsystem intake, ShooterSubsystem shooter) {
    addCommands(
        new FeedCommand(-60, feeder, shooter),
        new IndexCommand(50),
        new PulseIntakeCommand(intake));
  }
}
