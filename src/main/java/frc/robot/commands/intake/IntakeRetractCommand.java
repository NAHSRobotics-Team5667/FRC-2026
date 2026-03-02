package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRetractCommand extends ParallelCommandGroup {

  public IntakeRetractCommand(IntakeSubsystem intake) {
    addCommands(
        intake.setIntakeAngle(Degrees.of(IntakeConstants.INTAKE_UP_POSITION)), new IndexCommand(0));
  }
}
