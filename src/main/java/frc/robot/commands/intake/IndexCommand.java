package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IndexCommand extends Command {

  private IntakeSubsystem intake;
  private double percentOutput;

  public IndexCommand(double percentOutput) {
    intake = IntakeSubsystem.getInstance();
    this.percentOutput = percentOutput;
  }

  // Called when command is initiated/first scheduled
  @Override
  public void initialize() {
    intake.setIndexer(percentOutput);
  }

  // Called when scheduler runs while the command is scheduled
  @Override
  public void execute() {}

  // Called when the command is interruped or ended
  @Override
  public void end(boolean interrupted) {
    intake.setIndexer(0);
  }

  // Called so it should return true when the command will end
  @Override
  public boolean isFinished() {
    return false;
  }
}
