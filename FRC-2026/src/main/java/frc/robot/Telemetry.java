package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.ShooterSubsystem;

public class Telemetry {
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    private final ShooterSubsystem shooter;
    private final PIDController thetaController;

    public Telemetry(double maxSpeed, ShooterSubsystem shooter, PIDController thetaController) {
        MaxSpeed = maxSpeed;
        this.shooter = shooter;
        this.thetaController = thetaController;

        SignalLogger.start();

        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }
    

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Auto Align telemetry */
    private final NetworkTable alignTable = inst.getTable("AutoAlign");
    private final DoublePublisher thetaKpPub = alignTable.getDoubleTopic("kP").publish();
    private final DoublePublisher thetaKiPub = alignTable.getDoubleTopic("kI").publish();
    private final DoublePublisher thetaKdPub = alignTable.getDoubleTopic("kD").publish();
    private final DoublePublisher thetaSetpointPub = alignTable.getDoubleTopic("SetpointDeg").publish();
    private final DoublePublisher thetaErrorPub = alignTable.getDoubleTopic("ErrorDeg").publish();
    private final DoublePublisher thetaOutputPub = alignTable.getDoubleTopic("OutputRadPerSec").publish();
    private final DoublePublisher currentHeadingPub = alignTable.getDoubleTopic("CurrentHeadingDeg").publish();
    private final DoublePublisher desiredHeadingPub = alignTable.getDoubleTopic("DesiredHeadingDeg").publish();
    private final DoublePublisher omegaPub = alignTable.getDoubleTopic("OmegaCommand").publish();
    private final BooleanPublisher alignedPub = alignTable.getBooleanTopic("Aligned").publish();

    /* Shooter telemetry */
    private final NetworkTable shooterTable = inst.getTable("Shooter");
    private final DoublePublisher shooterTargetPub = shooterTable.getDoubleTopic("TargetRPM").publish();
    private final DoublePublisher shooterActualPub = shooterTable.getDoubleTopic("ActualRPM").publish();
    private final DoublePublisher shooterErrorPub = shooterTable.getDoubleTopic("ErrorRPM").publish();
    private final BooleanPublisher shooterAtSpeedPub = shooterTable.getBooleanTopic("AtSpeed").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }

        /* Auto Align Telemetry */

        double currentHeading = state.Pose.getRotation().getDegrees();

        currentHeadingPub.set(currentHeading);

        thetaKpPub.set(thetaController.getP());
        thetaKiPub.set(thetaController.getI());
        thetaKdPub.set(thetaController.getD());

        double errorRad = thetaController.getPositionError();
        thetaErrorPub.set(Math.toDegrees(errorRad));

        thetaSetpointPub.set(Math.toDegrees(thetaController.getSetpoint()));

        thetaOutputPub.set(thetaController.calculate(
                state.Pose.getRotation().getRadians()
        )); // optional if you want raw PID output view

        boolean aligned = Math.abs(errorRad) < Math.toRadians(2);
        alignedPub.set(aligned);

        /* Shooter Telemetry */

        double target = shooter.gettargetSpeed().in(RPM);
        double actual = shooter.getVelocity().in(RPM);
            
        shooterTargetPub.set(target);
        shooterActualPub.set(actual);
        shooterErrorPub.set(target - actual);
        shooterAtSpeedPub.set(shooter.atSpeedTrigger().getAsBoolean());
    }
}
