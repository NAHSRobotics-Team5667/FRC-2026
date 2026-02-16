// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class HubAlignCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;

    private final PIDController xController =
        new PIDController(3.0, 0, 0);

    private final PIDController yController =
        new PIDController(3.0, 0, 0);

    private final PIDController thetaController =
        new PIDController(4.0, 0, 0);

    private final SwerveRequest.FieldCentric autoAlignDrive =
        new SwerveRequest.FieldCentric()
            .withDeadband(0.0)
            .withRotationalDeadband(0.0);

    public HubAlignCommand(CommandSwerveDrivetrain drivetrain,
        Pose2d targetPose) {

        this.drivetrain = drivetrain;
        this.targetPose = targetPose;

        thetaController.enableContinuousInput(
            -Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {

    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        Pose2d current = drivetrain.getState().Pose;

        double xSpeed =
            xController.calculate(
                current.getX(),
                targetPose.getX());

        double ySpeed =
            yController.calculate(
                current.getY(),
                targetPose.getY());

        double omega =
            thetaController.calculate(
                current.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        drivetrain.applyRequest(() ->
            autoAlignDrive
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(omega)
        );
    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {

    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
            Pose2d current = drivetrain.getState().Pose;

        double positionError =
            current.getTranslation()
                .getDistance(targetPose.getTranslation());

        double angleError =
            Math.abs(current.getRotation()
                .minus(targetPose.getRotation())
                .getDegrees());

        return positionError < VisionConstants.AutoAlignTranslationTolerance && angleError < VisionConstants.AutoAlignAngleTolerance;
    }
}