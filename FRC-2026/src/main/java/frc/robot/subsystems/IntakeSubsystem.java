// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.gearing.GearBox;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig;

  private TalonFX deployMotor;
  private TalonFX rollerMotor;
  private TalonFX indexerMotor;

  VelocityVoltage m_request;

  private SmartMotorController talonMotorController;

  private ArmConfig armCfg;

  private Arm arm = new Arm(armCfg);

  public Command setIntakeAngle(Angle angle) {return arm.setAngle(angle);}
  
  public Command setIntakeDeploy(double dutycycle) { return arm.set(dutycycle);}

  public Command sysId() { return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}
  
  private static IntakeSubsystem instance = null;

  public IntakeSubsystem() {
    deployMotor = new TalonFX(IntakeConstants.INTAKE_DEPLOY);
    rollerMotor = new TalonFX(IntakeConstants.INTAKE_ROLLERS);
    indexerMotor = new TalonFX(IntakeConstants.INDEXER);

    smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(50,0,0,DegreesPerSecond.of(90),DegreesPerSecondPerSecond.of(45))
    .withSimClosedLoopController(50,0,0,DegreesPerSecond.of(90),DegreesPerSecondPerSecond.of(45))
    .withFeedforward(new ArmFeedforward(0,0,0))
    .withSimFeedforward(new ArmFeedforward(0,0,0))
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3,38/14)))
    .withMotorInverted(false)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40))
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25));

    m_request = new VelocityVoltage(0).withSlot(0);

    talonMotorController = new TalonFXWrapper(deployMotor, DCMotor.getKrakenX60Foc(1), smcConfig);

    armCfg = new ArmConfig(talonMotorController)
    .withSoftLimits(Degrees.of(-100), Degrees.of(20))
    .withHardLimit(Degrees.of(-120), Degrees.of(0))
    .withStartingPosition(Degrees.of(0))
    .withLength(Inches.of(8))
    .withMass(Pounds.of(8))
    .withTelemetry("Arm", TelemetryVerbosity.HIGH);

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

  // INTAKE DEPLOY --------------------------------------------

  public void setAngle(Angle angle) {
    arm.setAngle(angle);
  }

  public void setDutyCycle(double dutyCycle) {
    arm.set(dutyCycle);
  }

  public void setSysID() {
    arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
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
  }
}
