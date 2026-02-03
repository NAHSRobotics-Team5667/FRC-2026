package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;

public class ClimberSubsystem extends SubsystemBase {
    
  private TalonFX m_climber;
      // ========================================================
      // ============= CLASS & SINGLETON SETUP ==================
    
      // SINGLETON ----------------------------------------------
  
      private static ClimberSubsystem instance = null;
      private ClimberSubsystem() {
       // Initialize Shooter Motors
        m_climber = new TalonFX(ClimberConstants.CLIMB);
        m_climber.setNeutralMode(NeutralModeValue.Brake);
      }
    
      public static ClimberSubsystem getInstance() {
        if (instance == null) {
          instance = new ClimberSubsystem();
        }
    
        return instance;
      }  

      // ========================================================
      // ================== MOTOR ACTIONS =======================
    
      // CLIMBER ------------------------------------------------
     
      /**
       * Sets speed of the climbing motor. 0-100.
       * 
       * @param percentOutput % output for the motor.
       */
      public void set(double percentOutput) {
          double output = percentOutput / 100;    
          m_climber.set(output);    
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
