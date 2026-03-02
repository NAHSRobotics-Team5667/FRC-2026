package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;

public class ShooterCommands {

  public static Command autoAimShoot(Drive drive, ShooterSubsystem shooter) {
    return Commands.runEnd(
        () -> {
          Pose2d pose = drive.getPose();
          double distance =
              pose.getTranslation().getDistance(VisionConstants.getAllianceHubTranslation());

          if (distance < 0.5 || distance > 6.0 || Double.isNaN(distance)) {
            shooter.setShooterSpeed(RPM.of(ShooterConstants.DEFAULT_RPM));
          } else {
            // shooter.setDistanceBasedSpeed(distance);
            shooter.setShooterSpeed(RPM.of(ShooterConstants.DEFAULT_RPM));
          }
        },
        () -> {
          shooter.setShooterSpeed(RPM.of(ShooterConstants.DEFAULT_RPM));
        },
        shooter);
  }
}
