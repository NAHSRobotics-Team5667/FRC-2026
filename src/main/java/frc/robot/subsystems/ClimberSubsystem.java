package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private TalonFX m_climber;
  VelocityVoltage m_request;

  // ========================================================
  // ============= CLASS SETUP ==================

  public ClimberSubsystem() {
    // Climber Configuration for Velocity Control
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = ClimberConstants.CLIMB_KS;
    slot0Configs.kV = ClimberConstants.CLIMB_KV;
    slot0Configs.kP = ClimberConstants.CLIMB_KP;
    slot0Configs.kI = ClimberConstants.CLIMB_KI;
    slot0Configs.kD = ClimberConstants.CLIMB_KD;
    m_climber.getConfigurator().apply(slot0Configs);

    // Initialize Climber Motors
    m_climber = new TalonFX(ClimberConstants.CLIMB);
    m_climber.setNeutralMode(NeutralModeValue.Brake);
  }

  // ========================================================
  // ================== MOTOR ACTIONS =======================

  // CLIMBER ------------------------------------------------
  public void setVelocity(double velocity, double feedforward) {
    m_climber.setControl(m_request.withVelocity(velocity).withFeedForward(feedforward));
  }

  public double getVelocity() {
    return m_climber.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CLIMBER RPM", getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
