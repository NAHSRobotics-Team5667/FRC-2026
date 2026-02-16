package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;


public class VisionSubsystem extends SubsystemBase {
    private String front = VisionConstants.LIMELIGHT_FRONT_NAME;
    private String rear = VisionConstants.LIMELIGHT_REAR_NAME;

    private Pose2d bestPose = null;
    private double bestTimestamp = 0.0;
    private boolean hasMeasurement = false;
    private int tagCount = 0;

    //===============================================
    //============= LIMELIGHT METHODS ===============

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

    public double getTagID(String limelightName) {
        return LimelightHelpers.getFiducialID(limelightName);
    }

    public boolean tagInVision(String limelightName) {
        if (LimelightHelpers.getTV(limelightName)) {
            return true;
        } else {
            return false;
        }
    }

    
    @Override
    public void periodic() {

        var frontPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(front);
        var rearPoseEstimate  = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(rear);

        hasMeasurement = false;

        var best = chooseBest(frontPoseEstimate, rearPoseEstimate);

        if (best != null && best.tagCount > 0) {

            boolean closeEnough = best.avgTagDist < 5.0;
        
            if (closeEnough) {
                bestPose = best.pose;
                bestTimestamp = best.timestampSeconds;
                tagCount = best.tagCount;
                hasMeasurement = true;
            }
        }
        }
    
    private LimelightHelpers.PoseEstimate chooseBest(
            LimelightHelpers.PoseEstimate a,
            LimelightHelpers.PoseEstimate b) {

        if (a == null && b == null) return null;
        if (a == null) return b;
        if (b == null) return a;

        // Prefer more tags
        if (a.tagCount != b.tagCount)
            return a.tagCount > b.tagCount ? a : b;

        // If equal, prefer lower latency
        return a.latency < b.latency ? a : b;
    }

    public Optional<Pose2d> getEstimatedGlobalPose() {
        return hasMeasurement ? Optional.of(bestPose) : Optional.empty();
    }

    public double getTimestamp() {
        return bestTimestamp;
    }

    public int getTagCount() {
        return tagCount;
    }
}

