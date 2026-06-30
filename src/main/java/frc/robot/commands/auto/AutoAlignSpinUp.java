package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.shooter.PrepareShotCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;

public class AutoAlignSpinUp extends ParallelCommandGroup {

  public AutoAlignSpinUp(ShooterSubsystem shooter, Drive drive) {
    addCommands(
        DriveCommands.joystickDriveAtAngle(
            drive, () -> 0, () -> 0, () -> drive.getDirectionToHub()),
        new PrepareShotCommand(shooter, drive::getPose));
    ;
  }
}
