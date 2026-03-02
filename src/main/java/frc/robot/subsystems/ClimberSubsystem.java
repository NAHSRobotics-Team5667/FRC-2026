package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private TalonFX m_climber;

  // ========================================================
  // ============= CLASS SETUP ==================

  public ClimberSubsystem() {
    // Initialize Climber Motors
    m_climber = new TalonFX(ClimberConstants.CLIMB);
    m_climber.setNeutralMode(NeutralModeValue.Brake);
  }

  // ========================================================
  // ================== MOTOR ACTIONS =======================

  // CLIMBER ------------------------------------------------
  public void setPercentOutput(double percentOutput) {
    m_climber.set(percentOutput);
  }

  public double getPercentOutput() {
    return m_climber.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CLIMBER RPM", getPercentOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
