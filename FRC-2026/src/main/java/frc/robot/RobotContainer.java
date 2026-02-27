// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ClimberConstants.ClimbDirection;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.climber.ClimberCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.FeederCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(2.5);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(2.5);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final PIDController thetaController = new PIDController(4.0, 0, 0);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public static final VisionSubsystem vision = new VisionSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final ClimberSubsystem cllimber = ClimberSubsystem.getInstance();

  private final Telemetry logger = new Telemetry(MaxSpeed, shooter, thetaController);

  public RobotContainer() {
    configureBindings();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private double getRotationOutput() {
    // creates auto align mode w/ driver retaining translation control
    if (joystick.leftTrigger().getAsBoolean()) {
      Pose2d current = drivetrain.getState().Pose;
      Pose2d hub = VisionConstants.getAllianceHubPose();

      double dx = hub.getX() - current.getX();
      double dy = hub.getY() - current.getY();

      double desiredHeading = Math.atan2(dy, dx);

      double omega = thetaController.calculate(current.getRotation().getRadians(), desiredHeading);
      return MathUtil.clamp(omega, -MaxAngularRate, MaxAngularRate);
    } else {
      return -joystick.getRightX() * MaxAngularRate;
    }
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        m_xLimiter.calculate(joystick.getLeftY())
                            * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        m_yLimiter.calculate(joystick.getLeftX())
                            * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        getRotationOutput()) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    /* CONTROL SCHEME:

       Left Joystick - Swerve Translation
       Right Joystick - Swerve Rotation

       X - Reset field-centric heading
       A - Deploy/Retract Intake
       B - Outtake
       Y - Spin up shooter

       LT - Lock on to Hub + Calculate and command shooter RPM
       RT - Feed into Shooter
       LB - Auto-align climb left
       RB - Auto-align climb right

       Up - Climber Up
       Right - ?
       Down - Climber Down
       Left - ?

    */
    joystick.a().onTrue(new IntakeCommand());
    joystick.b().whileTrue(intake.setIntakeAngle(Degrees.of(40)));
    joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    joystick.y().whileTrue(shooter.setDutyCycle(ShooterConstants.SHOOTER_DUTY_CYCLE));
    joystick.leftTrigger().whileTrue(new ShooterCommand(shooter, drivetrain));
    joystick.rightTrigger().whileTrue(new FeederCommand(100));
    joystick.povUp().whileTrue(new ClimberCommand(ClimbDirection.UP));
    joystick.povDown().whileTrue(new ClimberCommand(ClimbDirection.DOWN));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    // Simple drive forward auton
    final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        // Then slowly drive forward (away from us) for 5 seconds.
        drivetrain
            .applyRequest(() -> drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
            .withTimeout(5.0),
        // Finally idle for the rest of auton
        drivetrain.applyRequest(() -> idle));
  }
}
