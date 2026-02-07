package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX m_shooter1;
    private TalonFX m_shooter2;
    private TalonFX m_feeder;
    private double targetRPM =  ShooterConstants.SHOOTER_MAX_RPM;
    private double targetFeederRPM = ShooterConstants.FEEDER_MAX_RPM;
    private SmartMotorControllerConfig smcConfig;
    private SmartMotorController shooterMotorController;
    private FlyWheelConfig shooterConfig;
    private FlyWheel shooter;
    
    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================
    
    // SINGLETON ----------------------------------------------
    private static ShooterSubsystem instance = null;
    
    private ShooterSubsystem() {
        SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(ShooterConstants.SHOOTER_KP, ShooterConstants.SHOOTER_KI, ShooterConstants.SHOOTER_KD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(ShooterConstants.SHOOTER_KP, ShooterConstants.SHOOTER_KI, ShooterConstants.SHOOTER_KD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(ShooterConstants.SHOOTER_KS, ShooterConstants.SHOOTER_KV, ShooterConstants.SHOOTER_KA))
        .withSimFeedforward(new SimpleMotorFeedforward(ShooterConstants.SHOOTER_KS, ShooterConstants.SHOOTER_KV, ShooterConstants.SHOOTER_KA))
        // Telemetry name and verbosity level
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        .withGearing(24/22)
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

        shooterMotorController = new TalonFXWrapper(m_shooter1, DCMotor.getKrakenX60(1), smcConfig);

        shooterConfig = new FlyWheelConfig(shooterMotorController)
        .withMOI(MomentOfInertia.ofBaseUnits(0.0058527931, KilogramSquareMeters))
        // Max Speed
        .withUpperSoftLimit(RPM.of(3000))
        // Telemetry Name + Verbosity
        .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

        shooter = new FlyWheel(shooterConfig);

        m_feeder = new TalonFX(ShooterConstants.FEEDER);
        m_feeder.setNeutralMode(NeutralModeValue.Coast);

        m_shooter2.setControl(new Follower(m_shooter1.getDeviceID(), null));
    }
    
    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
    
        return instance;
    }
    
    // ========================================================
    // ================== MOTOR ACTIONS =======================
    
    // SHOOTER ------------------------------------------------

    /**
     * Sets shooter speed
     * @param speed
     */
    public void setShooterSpeed(AngularVelocity speed) {
        shooter.setSpeed(speed);
    }
    
    public void setFeeder(double percentOutput) {
        double output = percentOutput / 100;
        targetFeederRPM = output * ShooterConstants.SHOOTER_MAX_RPM;

        m_feeder.set(output);
    }

    /**
     * Sets the dutycycle of the shooter
     * 
     * @param dutyCycle DutyCycle to set
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     * 
     */
    public Command setDutyCycle(double DutyCycle) {return shooter.set(DutyCycle);}

    /**
    * @return Target RPM of the main shooter.
    */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
    * @return Target RPM of the feeder.
    */
    public double getTargetFeederRPM() {
        return targetFeederRPM;
    }

    /**
    * @return RPM of the feeder.
    */
    public double getFeederRPM() {
        double speed = m_feeder.getVelocity().getValueAsDouble();
        return speed;
    }

    /**
     * Gets the current velocity of the shooter
     * 
     * @return Shooter velocity
     * 
     */
    public AngularVelocity getVelocity() {return shooter.getSpeed();}

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
