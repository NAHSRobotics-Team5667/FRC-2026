package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    // Basic targeting data
    double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

    double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
    double tync = LimelightHelpers.getTYNC("");  // Vertical offset from principal pixel/point to target in degrees 
    
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Set a custom crop window for improved performance (-1 to 1 for each value)
        LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);

        // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        LimelightHelpers.setCameraPose_RobotSpace("",
            0.5,    // Forward offset (meters)
            0.0,    // Side offset (meters)
            0.5,    // Height offset (meters)
            0.0,    // Roll (degrees)
            30.0,   // Pitch (degrees)
            0.0     // Yaw (degrees)
        );

        // Set AprilTag offset tracking point (meters)
        LimelightHelpers.setFiducial3DOffset("",
            0.0,    // Forward offset
            0.0,    // Side offset
            0.5     // Height offset
        );

        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{1, 2, 3, 4}); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride("", 2.0f); // Process at half resolution

        // Adjust keystone crop window (-0.95 to 0.95 for both horizontal and vertical)
        LimelightHelpers.setKeystone("", 0.1, -0.05);
    }

    

    @Override
    public void periodic() {
        updateFromLimelight("limelight-front");
        updateFromLimelight("limelight-rear");


    }

    private void updateFromLimelight(String name) {
        if (!LimelightHelpers.getTV(name)) return;

        Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(name);
        double timestamp = Timer.getFPGATimestamp()
                - LimelightHelpers.getLatency_Capture(name) / 1000
                - LimelightHelpers.getLatency_Pipeline(name) / 1000;

        drivetrain.addVisionMeasurement(pose, timestamp);
    }
}
