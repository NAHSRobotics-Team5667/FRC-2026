package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRollCommand extends Command {

  private IntakeSubsystem intake;

  public IntakeRollCommand() {
    intake = IntakeSubsystem.getInstance();
  }

  // Called when command is initiated/first scheduled
  @Override
  public void initialize() {
    intake.setRollerVelocity(0.000, 0.00);
  }

  // Called when scheduler runs while the command is scheduled
  @Override
  public void execute() {
    intake.setRollerVelocity(IntakeConstants.ROLLER_VELOCITY, IntakeConstants.ROLLER_FEEDFORWARD);
  }

  // Called when the command is interruped or ended
  @Override
  public void end(boolean interrupted) {
    intake.setRollerVelocity(0.000, 0.00);
  }

  // Called so it should return true when the command will end
  @Override
  public boolean isFinished() {
    return false;
  }
}
