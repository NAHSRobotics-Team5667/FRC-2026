package frc.robot.commands.intake;

import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.util.States.IntakeState;

public class IntakeDeployCommand extends Command {

    private IntakeSubsystem intake;
    private StateManager stateManager;
    private IntakeState mode;

    public IntakeDeployCommand(Angle angle) {
        intake = IntakeSubsystem.getInstance();
        stateManager = StateManager.getInstance();
        // addRequirement() - prevent two commands from being run at the same time
        addRequirements(intake);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        mode = stateManager.getIntakeState();
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        if (mode == IntakeState.DEPLOYED) {

        } else {
            
        }
    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {

    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false;
    }
}