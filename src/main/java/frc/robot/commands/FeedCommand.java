package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedCommand extends Command {

  private final FeederSubsystem feeder;
  private final double percentOutput;
  private final ShooterSubsystem shooter;

  public FeedCommand(double percentOutput, FeederSubsystem feeder, ShooterSubsystem shooter) {
    this.feeder = feeder;
    this.percentOutput = percentOutput;
    this.shooter = shooter;

    addRequirements(feeder);
  }

  @Override
  public void execute() {
    // if (shooter.atSpeedTrigger().getAsBoolean()) {
    feeder.setFeeder(percentOutput);
    // } else {
    //  feeder.setFeeder(0);
    // }
  }

  @Override
  public void end(boolean interrupted) {
    feeder.setFeeder(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
