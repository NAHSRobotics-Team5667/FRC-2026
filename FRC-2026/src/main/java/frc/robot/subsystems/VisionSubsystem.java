package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
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
