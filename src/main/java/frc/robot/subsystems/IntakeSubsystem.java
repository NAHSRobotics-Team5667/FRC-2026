// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX deployMotor = new TalonFX(IntakeConstants.INTAKE_DEPLOY);
  private TalonFX rollerMotor = new TalonFX(IntakeConstants.INTAKE_ROLLERS);
  private TalonFX indexerMotor = new TalonFX(IntakeConstants.INDEXER);

  private static IntakeSubsystem instance = null;

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              25, 0, 1, DegreesPerSecond.of(1500), DegreesPerSecondPerSecond.of(1000))
          .withSimClosedLoopController(
              20, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(180))
          .withFeedforward(new ArmFeedforward(0.1, 0.2, 0.5))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(38.00 / 14.00, 12)))
          .withMotorInverted(true)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40));

  private VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  private SmartMotorController talonMotorController =
      new TalonFXWrapper(deployMotor, DCMotor.getKrakenX60Foc(1), smcConfig);

  private ArmConfig armCfg =
      new ArmConfig(talonMotorController)
          .withStartingPosition(Degrees.of(IntakeConstants.INTAKE_UP_POSITION))
          .withLength(Inches.of(8))
          .withMass(Pounds.of(4))
          .withTelemetry("Intake", TelemetryVerbosity.HIGH);

  private Arm arm = new Arm(armCfg);

  public Command setIntakeAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  public Command setIntakeDeploy(double dutycycle) {
    return arm.set(dutycycle);
  }

  public void setIntakeDeployMotor(double dutycycle) {
    talonMotorController.setDutyCycle(dutycycle);
  }

  public Command sysId() {
    return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  public Command resetIntakeEncoder() {
    return Commands.sequence(
        // Drive intake downward into hardstop
        setIntakeDeploy(-4).withTimeout(1),
        setIntakeDeploy(0).withTimeout(0.1),
        this.runOnce(
            () -> {
              talonMotorController.setEncoderPosition(
                  Degrees.of(Constants.IntakeConstants.INTAKE_DOWN_POSITION));
            }));
  }

  public IntakeSubsystem() {
    // Intake Roller Configuration for Velocity Control
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = IntakeConstants.ROLLERKS;
    slot0Configs.kV = IntakeConstants.ROLLERKV;
    slot0Configs.kP = IntakeConstants.ROLLERKP;
    slot0Configs.kI = IntakeConstants.ROLLERKI;
    slot0Configs.kD = IntakeConstants.ROLLERKD;
    rollerMotor.getConfigurator().apply(slot0Configs);
  }

  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }

    return instance;
  }

  // ========================================================
  // ================== MOTOR ACTIONS =======================

  // ROLLERS + INDEXER --------------------------------------

  public void setRollerVelocity(double velocity, double feedforward) {
    rollerMotor.setControl(m_request.withVelocity(velocity).withFeedForward(feedforward));
  }

  public void setIndexer(double percentOutput) {
    double output = percentOutput / 100;
    indexerMotor.set(output);
  }

  public void setArmAngle(Angle angle) {
    talonMotorController.setPosition(angle);
  }

  public double getRollerVelocity() {
    return rollerMotor.getVelocity().getValueAsDouble();
  }

  public double getIndexerSpeed() {
    return indexerMotor.getVelocity().getValueAsDouble();
  }

  // ========================================================
  // ================= STATE MANAGEMENT ======================

  public String getState() {
    if (getRollerVelocity() > 0) {
      return "DEPLOYED";
    } else {
      return "RETRACTED";
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    arm.simIterate();
    SmartDashboard.putString("[INTAKE] Status", getState());
  }
}
