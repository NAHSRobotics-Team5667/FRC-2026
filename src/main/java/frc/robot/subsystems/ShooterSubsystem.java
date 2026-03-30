package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX m_shooter1 = new TalonFX(ShooterConstants.SHOOTER_1);
  private TalonFX m_shooter2 = new TalonFX(ShooterConstants.SHOOTER_2);
  private TalonFX m_shooter3 = new TalonFX(ShooterConstants.SHOOTER_3);
  private TalonFX m_shooter4 = new TalonFX(ShooterConstants.SHOOTER_4);

  private AngularVelocity targetSpeed = RPM.of(1000);

  // ========================================================
  // ============= CLASS SETUP ==================

  SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              ShooterConstants.SHOOTER_KP,
              ShooterConstants.SHOOTER_KI,
              ShooterConstants.SHOOTER_KD,
              DegreesPerSecond.of(50000000),
              DegreesPerSecondPerSecond.of(50000000))
          .withSimClosedLoopController(
              ShooterConstants.SHOOTER_KP,
              ShooterConstants.SHOOTER_KI,
              ShooterConstants.SHOOTER_KD,
              DegreesPerSecond.of(90),
              DegreesPerSecondPerSecond.of(45))
          // Feedforward Constants
          .withFeedforward(
              new SimpleMotorFeedforward(
                  ShooterConstants.SHOOTER_KS,
                  ShooterConstants.SHOOTER_KV,
                  ShooterConstants.SHOOTER_KA))
          .withSimFeedforward(
              new SimpleMotorFeedforward(
                  ShooterConstants.SHOOTER_KS,
                  ShooterConstants.SHOOTER_KV,
                  ShooterConstants.SHOOTER_KA))
          // Telemetry name and verbosity level
          .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          .withGearing(24.0 / 22)
          // Motor properties to prevent over currenting.
          .withMotorInverted(true)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(60))
          .withClosedLoopRampRate(Seconds.of(0.001))
          .withOpenLoopRampRate(Seconds.of(0.001))
          .withFollowers(
              Pair.of(m_shooter2, false), Pair.of(m_shooter3, true), Pair.of(m_shooter4, true));

  private SmartMotorController shooterMotorController =
      new TalonFXWrapper(m_shooter1, DCMotor.getKrakenX60(2), smcConfig);

  private final FlyWheelConfig shooterConfig =
      new FlyWheelConfig(shooterMotorController)
          .withMOI(MomentOfInertia.ofBaseUnits(0.0058527931, KilogramSquareMeters))
          // Max Speed
          .withUpperSoftLimit(RPM.of(4000))
          // Telemetry Name + Verbosity
          .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  private FlyWheel shooter = new FlyWheel(shooterConfig);

  public ShooterSubsystem() {}

  private final Trigger atSpeed =
      new Trigger(
          () -> {
            if (targetSpeed.magnitude() == 0) {
              return false;
            }
            return shooterMotorController.getMechanismVelocity().isNear(targetSpeed, RPM.of(80));
          });
  // ========================================================
  // ================== MOTOR ACTIONS =======================

  public void setShooterSpeed(AngularVelocity speed) {
    shooterMotorController.setVelocity(speed);
    targetSpeed = speed;
  }

  public AngularVelocity getShooterVelocity() {
    return shooterMotorController.getMechanismVelocity();
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return shooter.setSpeed(speed);
  }

  public Command setVelocity(AngularVelocity speed) {
    targetSpeed = speed;
    return shooter.setSpeed(speed);
  }

  public AngularVelocity gettargetSpeed() {
    return targetSpeed;
  }

  public AngularVelocity getCurrentRPM() {
    return shooter.getSpeed();
  }

  /**
   * Gets the current velocity of the shooter
   *
   * @return Shooter velocity
   */
  public AngularVelocity getVelocity() {
    return shooter.getSpeed();
  }

  public Trigger atSpeedTrigger() {
    return atSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();
  }
}
