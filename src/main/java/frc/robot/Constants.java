// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

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

    public static final double ROLLERKV = 1;
    public static final double ROLLERKS = 0;
    public static final double ROLLERKP = 0;
    public static final double ROLLERKI = 0;
    public static final double ROLLERKD = 0;

    public static final double ROLLER_VELOCITY = 1500;
    public static final double ROLLER_FEEDFORWARD = 0;

    public static final double INTAKE_DOWN_POSITION = 65;
    public static final double INTAKE_UP_POSITION = 143.170126;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_1 = 51;
    public static final int SHOOTER_2 = 52;
    public static final int FEEDER = 53;

    public static final double SHOOTER_MAX_RPM = 1;
    public static final double FEEDER_MAX_RPM = 1;

    public static final double SHOOTER_KP = 50;
    public static final double SHOOTER_KI = 0;
    public static final double SHOOTER_KD = 0;
    public static final double SHOOTER_KS = 0;
    public static final double SHOOTER_KV = 0;
    public static final double SHOOTER_KA = 0;

    // placeholders
    public static final double[][] DISTANCE_RPM_MAP = {
      {1.5, 2400},
      {2.0, 2600},
      {2.5, 2800},
      {3.0, 3000}
    };

    public static final double DEFAULT_RPM = 1500;
    public static final double RPM_TOLERANCE = 50;
  }

  public static class ClimberConstants {
    public static final int CLIMB = 61;
    public static final double CLIMB_FEEDFORWARD = 1;
    public static final double CLIMB_PERCENT_OUTPUT = 0.75;

    public enum ClimbDirection {
      UP,
      DOWN
    }
  }

  public static class VisionConstants {
    public static final String LIMELIGHT_FRONT_NAME = "limelight-front";
    public static final String LIMELIGHT_REAR_NAME = "limelight-rear";

    public static final Translation2d BLUE_HUB_POSE = new Translation2d(4.6192143328, 4.0378468646);

    public static final Translation2d RED_HUB_POSE = new Translation2d(11.9080899252, 4.0394513572);

    public static final double AutoAlignTranslationTolerance = 0.1; // meters
    public static final double AutoAlignAngleTolerance = 5; // degrees

    public static Translation2d getAllianceHubTranslation() {
      return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
          ? VisionConstants.BLUE_HUB_POSE
          : VisionConstants.RED_HUB_POSE;
    }
  }

  public static final class SwerveConstants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }
  }
}
