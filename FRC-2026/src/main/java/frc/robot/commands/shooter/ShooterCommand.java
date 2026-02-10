// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import static edu.wpi.first.units.Units.RPM;


public class ShooterCommand extends Command {

    private ShooterSubsystem shooter;
    private AngularVelocity shooterSpeed = RPM.of(ShooterConstants.SHOOTER_MAX_RPM);

    /**
     * Creates a new ShootCommand.
     * 
     * @param speed Speed the shooter runs at
     */
    public ShooterCommand(AngularVelocity speed) {
        shooter = ShooterSubsystem.getInstance();
        shooterSpeed = speed;


        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        shooter.setShooterSpeed(RPM.of(0));
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        shooter.setShooterSpeed(shooterSpeed);
    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        shooter.setShooterSpeed(RPM.of(0));
    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false;
    }
}