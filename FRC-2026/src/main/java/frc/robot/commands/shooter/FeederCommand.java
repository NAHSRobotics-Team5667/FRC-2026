// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class FeederCommand extends Command {

    private ShooterSubsystem shooter;
    private double speed = 100;

    /**
     * Creates a new ShootCommand.
     * 
     * @param percentSpeed Speed the feeder runs at
     */
    public FeederCommand(double speed) {
        shooter = ShooterSubsystem.getInstance();
        this.speed = speed;


        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        shooter.setFeeder(0.00);
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        shooter.setFeeder(speed);
    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        shooter.setFeeder(0);
    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false;
    }
}