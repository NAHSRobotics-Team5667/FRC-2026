// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class HubAlignCommand extends Command {

    private ShooterSubsystem shooter;
    private VisionSubsystem limelight;


    /**
     * Creates a new ShootCommand.
     * 
     * @param percentSpeed Speed the feeder runs at
     */
    public HubAlignCommand(CommandSwerveDrivetrain drivetrain) {
        shooter = ShooterSubsystem.getInstance();
        limelight = new VisionSubsystem(drivetrain);


        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {

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