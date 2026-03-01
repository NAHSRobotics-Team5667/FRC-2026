package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDeployCommand extends ParallelCommandGroup {

    public IntakeDeployCommand(IntakeSubsystem intake){
        addCommands(
            intake.setIntakeAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_POSITION)),
            new IntakeRollCommand(),
            new IndexCommand()
        );
    }
}
