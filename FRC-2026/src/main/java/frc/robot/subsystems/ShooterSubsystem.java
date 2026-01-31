package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX m_shooter1;
    private TalonFX m_shooter2;
    private double targetRPM =  ShooterConstants.SHOOTER_MAX_RPM;

    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------
    private static ShooterSubsystem instance = null;

    private ShooterSubsystem() {
        // Initialize Shooter Motors
        m_shooter1 = new TalonFX(ShooterConstants.SHOOTER_1);
        m_shooter1.setNeutralMode(NeutralModeValue.Coast);

        m_shooter2 = new TalonFX(ShooterConstants.SHOOTER_2);
        m_shooter2.setNeutralMode(NeutralModeValue.Coast);
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
     * Sets speed of the shooter for both sets of wheels. 0-100.
     * 
     * @param percentOutput % output for both motors.
     */
    public void set(double percentOutput) {
        double output = percentOutput / 100;
        targetRPM = output * ShooterConstants.SHOOTER_MAX_RPM;

        m_shooter1.set(output);
        m_shooter2.set(output);

    }

    /**
    * @return RPM of the first set of wheels.
    */
    public double getShooter1RPM() {
        double speed = m_shooter1.getVelocity().getValueAsDouble();
        return speed;
    }

     /**
     * @return RPM of the second set of wheels.
     */
    public double getShooter2RPM() {
        double speed = m_shooter2.getVelocity().getValueAsDouble();
        return speed;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

     @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    } 
}
