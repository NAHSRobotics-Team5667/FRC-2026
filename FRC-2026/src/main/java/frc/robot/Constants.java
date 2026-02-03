// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeConstants {
    public static final int INTAKE_DEPLOY = 41;
    public static final int INTAKE_ROLLERS = 42;
    public static final int INDEXER = 43;

    public static final double ROLLERKS = 1;
    public static final double ROLLERKV = 1;
    public static final double ROLLERKP = 1;
    public static final double ROLLERKI = 1;
    public static final double ROLLERKD = 1;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_1 = 51;
    public static final int SHOOTER_2 = 52;
    public static final int FEEDER = 53;

    public static final double SHOOTER_MAX_RPM = 1;
    public static final double FEEDER_MAX_RPM = 1;

    public static final double SHOOTER_KP = 1;
    public static final double SHOOTER_KI = 1;
    public static final double SHOOTER_KD = 1;
    public static final double SHOOTER_KS = 1;
    public static final double SHOOTER_KV = 1;
    public static final double SHOOTER_KA = 1;
  }

  public static class ClimberConstants {
    public static final int CLIMB = 61;
    public static final double CLIMB_DUTYCYCLE = 1;
  }

  public static class SwerveConstants {
    public static final double turnkP = 100;
    public static final double turnkI = 0;
    public static final double turnkD = 0.5;
  }
  
}
