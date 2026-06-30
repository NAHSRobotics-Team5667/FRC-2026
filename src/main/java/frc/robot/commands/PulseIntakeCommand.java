package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class PulseIntakeCommand extends Command {

  private final IntakeSubsystem intake;

  private final Timer rollerTimer = new Timer();
  private final Timer pivotTimer = new Timer();

  public PulseIntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    rollerTimer.reset();
    rollerTimer.start();
    pivotTimer.reset();
    pivotTimer.start();
  }

  @Override
  public void execute() {

    // -------------------------
    // Intake Roller Pulsing
    // -------------------------

    double tRoller = rollerTimer.get();

    if ((tRoller % 1) < 0.5) {
      intake.setRollerVelocity(
          -0.8 * IntakeConstants.ROLLER_VELOCITY, IntakeConstants.ROLLER_FEEDFORWARD);
      ;
    } else {
      intake.setRollerVelocity(IntakeConstants.ROLLER_VELOCITY, IntakeConstants.ROLLER_FEEDFORWARD);
      ;
    }

    double tPivot = pivotTimer.get();

    if ((tPivot % 2) < 0.5) {
      intake.setIntakeDeployMotor(12);
      ;
    } else {
      intake.setIntakeDeployMotor(-0.3);
      ;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.setRollerVelocity(0, 0);
    intake.setArmAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_POSITION));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
