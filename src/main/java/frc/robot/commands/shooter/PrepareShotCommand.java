package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PoseConstants;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

public class PrepareShotCommand extends Command {
  private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap =
      new InterpolatingTreeMap<>(
          (startValue, endValue, q) ->
              InverseInterpolator.forDouble()
                  .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
          (startValue, endValue, t) ->
              new Shot(
                  Interpolator.forDouble()
                      .interpolate(startValue.shooterRPM, endValue.shooterRPM, t)));

  static {
    distanceToShotMap.put(Inches.of(96.092), new Shot(2200));
    distanceToShotMap.put(Inches.of(158.327), new Shot(2680));
    distanceToShotMap.put(Inches.of(53.076), new Shot(1980));
  }

  private final ShooterSubsystem shooter;
  private final Supplier<Pose2d> robotPoseSupplier;

  public PrepareShotCommand(ShooterSubsystem shooter, Supplier<Pose2d> robotPoseSupplier) {
    this.shooter = shooter;
    this.robotPoseSupplier = robotPoseSupplier;
    addRequirements(shooter);
  }

  public boolean isReadyToShoot() {
    return shooter.atSpeedTrigger().getAsBoolean();
  }

  private Distance getDistanceToHub() {
    final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
    final Translation2d hubPosition = PoseConstants.hubPosition();
    return Meters.of(robotPosition.getDistance(hubPosition));
  }

  @Override
  public void execute() {
    final Distance distanceToHub = getDistanceToHub();
    final Shot shot = distanceToShotMap.get(distanceToHub);
    shooter.setShooterSpeed(RPM.of(shot.shooterRPM));
    SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterSpeed(RPM.of(0));
  }

  public static class Shot {
    public final double shooterRPM;

    public Shot(double shooterRPM) {
      this.shooterRPM = shooterRPM;
    }
  }
}
