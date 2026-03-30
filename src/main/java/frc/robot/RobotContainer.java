// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.PulseIntakeCommand;
import frc.robot.commands.intake.IndexCommand;
import frc.robot.commands.intake.IntakeDeployCommand;
import frc.robot.commands.intake.IntakeRollCommand;
import frc.robot.commands.shooter.PrepareShotCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  // public final ClimberSubsystem climber = new ClimberSubsystem();
  public final FeederSubsystem feeder = new FeederSubsystem();
  private final Vision vision;

  private final SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(3);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.SwerveConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    NamedCommands.registerCommand("Auto Start", new IntakeDeployCommand(intake).withTimeout(6));

    NamedCommands.registerCommand(
        "Depot Scoring",
        new ParallelCommandGroup(
                new PrepareShotCommand(shooter, drive::getPose),
                new FeedCommand(-90, feeder, shooter),
                new IndexCommand(30),
                new PulseIntakeCommand(intake))
            .withTimeout(8));

    NamedCommands.registerCommand("Leave for Neutral", new IntakeDeployCommand(intake));

    // Set up SysId routines
    autoChooser.addOption("Depot and Collect", new PathPlannerAuto("Depot and Collect"));
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -xSlewRateLimiter.calculate(controller.getLeftY()),
            () -> -ySlewRateLimiter.calculate(controller.getLeftX()),
            () -> -controller.getRightX()));

    // intake.setDefaultCommand(
    //    intake.setIntakeAngle(Degrees.of(IntakeConstants.INTAKE_CARRY_POSITION)));

    shooter.setDefaultCommand(shooter.setVelocity(RPM.of(0)));

    // Lock to hub when RT button is held
    controller
        .rightTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> drive.getDirectionToHub()));

    // Switch to X pattern(brake) when B button is pressed
    // controller.b().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when X button is pressed
    controller
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller.b().toggleOnTrue(new IntakeDeployCommand(intake));

    controller.rightTrigger().whileTrue(new PrepareShotCommand(shooter, drive::getPose));

    controller
        .leftBumper()
        .whileTrue(
            new ParallelCommandGroup(
                new IntakeRollCommand(IntakeConstants.ROLLER_VELOCITY),
                new IndexCommand(20),
                intake.setIntakeAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_POSITION))))
        .onFalse(intake.setIntakeAngle(Degrees.of(IntakeConstants.INTAKE_UP_POSITION)));

    controller
        .rightBumper()
        .toggleOnTrue(intake.setIntakeAngle(Degrees.of(IntakeConstants.INTAKE_UP_POSITION)));

    controller
        .a()
        .whileTrue(
            new ParallelCommandGroup(
                new FeedCommand(-90, feeder, shooter),
                new IndexCommand(60),
                new PulseIntakeCommand(intake)));

    // controller.povUp().whileTrue(new ClimberCommand(ClimbDirection.UP, climber));
    // controller.povDown().whileTrue(new ClimberCommand(ClimbDirection.DOWN, climber));

    controller
        .povLeft()
        .whileTrue(
            new ParallelCommandGroup(new FeedCommand(-60, feeder, shooter), new IndexCommand(50)));

    controller.y().onTrue(intake.resetIntakeEncoder());

    controller
        .rightTrigger()
        .and(shooter.atSpeedTrigger())
        .whileTrue(
            Commands.run(
                () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.75)));

    controller
        .rightTrigger()
        .onFalse(
            Commands.runOnce(
                () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)));

    controller
        .rightBumper()
        .onTrue(new InstantCommand(() -> drive.setPose(Constants.PoseConstants.blueHubBasePose)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
