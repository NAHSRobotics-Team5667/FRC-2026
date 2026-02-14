package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

    }

    //===============================================
    //============= LIMELIGHT METHODS ===============
    

    private void updateFromLimelight(String name) {
        if (!LimelightHelpers.getTV(name)) return;

        Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(name);
        double timestamp = Timer.getFPGATimestamp()
                - LimelightHelpers.getLatency_Capture(name) / 1000
                - LimelightHelpers.getLatency_Pipeline(name) / 1000;

        drivetrain.addVisionMeasurement(pose, timestamp);
    }

    public double getDistanceToTag(double targetHeight) {
        Rotation2d angleToGoal = Rotation2d.fromDegrees(0)
        .plus(Rotation2d.fromDegrees(LimelightHelpers.getTX("limelight-front")));
        double distance = (targetHeight - 0) / angleToGoal.getTan();
        return distance; //Replace zeroes with the degree the limelight is mounted at + height off from the ground
    }

    public double getRotationToTag(double targetHeight) {
        double cameraLensHorizontalOffset = LimelightHelpers.getTX("limelight") / getDistanceToTag(targetHeight);
        double realHorizontalOffset = Math.atan(cameraLensHorizontalOffset / getDistanceToTag(targetHeight));
        double rotationError = Math.atan(realHorizontalOffset / getDistanceToTag(targetHeight));
        return rotationError;
    }


    

    

    @Override
    public void periodic() {
        updateFromLimelight("limelight-front");
        updateFromLimelight("limelight-rear");


    }
}
