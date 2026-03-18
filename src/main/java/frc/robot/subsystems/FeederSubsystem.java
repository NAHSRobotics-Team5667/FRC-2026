// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

  private final TalonFX m_feeder = new TalonFX(Constants.ShooterConstants.FEEDER);

  public FeederSubsystem() {}

  public void setFeeder(double percentOutput) {
    double output = percentOutput / 100;
    m_feeder.set(output);
  }
}
