package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {

  private ClimberConstants.ClimbDirection direction;
  private final ClimberSubsystem climber;
  private double percentOutput;

  public ClimberCommand(ClimberConstants.ClimbDirection direction, ClimberSubsystem climber) {
    // addRequirement() - prevent two commands from being run at the same time
    this.direction = direction;
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when command is initiated/first scheduled
  @Override
  public void initialize() {
    if (direction.equals(ClimberConstants.ClimbDirection.DOWN)) {
      percentOutput = -ClimberConstants.CLIMB_PERCENT_OUTPUT;
    } else {
      percentOutput = ClimberConstants.CLIMB_PERCENT_OUTPUT;
    }
  }

  // Called when scheduler runs while the command is scheduled
  @Override
  public void execute() {
    climber.setPercentOutput(percentOutput);
  }

  // Called when the command is interruped or ended
  @Override
  public void end(boolean interrupted) {
    climber.setPercentOutput(0);
  }

  // Called so it should return true when the command will end
  @Override
  public boolean isFinished() {
    return false;
  }
}
