package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class FeedCommand extends Command {

  private final FeederSubsystem feeder;
  private final double percentOutput;

  public FeedCommand(double percentOutput, FeederSubsystem feeder) {
    this.feeder = feeder;
    this.percentOutput = percentOutput;

    addRequirements(feeder);
  }

  @Override
  public void initialize() {
    feeder.setFeeder(percentOutput);
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
