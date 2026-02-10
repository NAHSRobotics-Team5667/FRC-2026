package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.util.States.IntakeState;
import frc.robot.util.States.ClimberState;

public class StateManager extends SubsystemBase {
    private IntakeState intakeState; 
    private ClimberState climberState;

    private static StateManager instance = null;

    private StateManager() {
        intakeState = IntakeState.RETRACTED;
        climberState = ClimberState.RETRACTED;
    }

    public static StateManager getInstance() {
        if (instance == null) {
            instance = new StateManager();
        }

       return instance;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public ClimberState getClimberState() {
        return climberState;
    }

}
