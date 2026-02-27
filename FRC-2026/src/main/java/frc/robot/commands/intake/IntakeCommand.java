package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeArmState;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  private IntakeSubsystem intake;
  private IntakeArmState armState;
  /*
   * If Intake Retracted on Command Start: Deploy and immediately start rollers + index
   * If Intake Deployed on Command Start: Turn off motors THEN retract
   */
  public IntakeCommand() {
    intake = IntakeSubsystem.getInstance();

    // addRequirement() - prevent two commands from being run at the same time
    addRequirements(intake);
  }

  // Called when command is initiated/first scheduled
  @Override
  public void initialize() {
    armState = intake.getState();
  }

  // Called when scheduler runs while the command is scheduled
  @Override
  public void execute() {
    if (armState.equals(IntakeArmState.RETRACTED)) {
      intake.setArmAngle(Degrees.of(40));
      intake.setRollerVelocity(IntakeConstants.ROLLER_VELOCITY, IntakeConstants.ROLLER_FEEDFORWARD);
      intake.setIndexer(100);
    } else {
      intake.setRollerVelocity(0.000, 0.00); // Turn off Motors first to be safe
      intake.setIndexer(0);
      intake.setArmAngle(Degrees.of(0));
    }
  }

  // Called when the command is interruped or ended
  @Override
  public void end(boolean interrupted) {
    if (armState.equals(IntakeArmState.RETRACTED)) {
      intake.changeArmState(true);
    } else {
      intake.changeArmState(false);
    }
  }

  // Called so it should return true when the command will end
  @Override
  public boolean isFinished() {
    return false;
  }
}
