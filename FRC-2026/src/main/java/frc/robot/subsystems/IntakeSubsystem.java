package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX m_deploy;
  private TalonFX m_rollers;
  private TalonFX m_indexer;

  // ========================================================
  // ============= CLASS & SINGLETON SETUP ==================
    
  // SINGLETON ----------------------------------------------
  private static IntakeSubsystem instance = null;

  private IntakeSubsystem() {
    m_deploy = new TalonFX(IntakeConstants.INTAKE_DEPLOY);
    m_rollers = new TalonFX(IntakeConstants.INTAKE_ROLLERS);
    m_indexer = new TalonFX(IntakeConstants.INDEXER);
  }

  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    
    return instance;
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
