package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
public class ClimberCommand extends Command {

    private String direction;
    private ClimberSubsystem climber;
    private double velocity;

    public ClimberCommand(String direction) {
        // addRequirement() - prevent two commands from being run at the same time
        this.direction = direction;
        climber = ClimberSubsystem.getInstance();
        addRequirements(climber);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        if (direction.equals("down")) {
            velocity = -ClimberConstants.CLIMB_SPEED;
        } else {
            velocity = ClimberConstants.CLIMB_SPEED;
        }
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {

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