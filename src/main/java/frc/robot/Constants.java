// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

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

    public static final double ROLLER_VELOCITY = 90;
    public static final double ROLLER_FEEDFORWARD = 0;

    public static final double INTAKE_DOWN_POSITION = 60;
    public static final double INTAKE_CARRY_POSITION = 125;
    public static final double INTAKE_UP_POSITION = 140;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_1 = 51; // leftside
    public static final int SHOOTER_2 = 52; // leftside
    public static final int SHOOTER_3 = 53; // rightside
    public static final int SHOOTER_4 = 54; // rightside
    public static final int FEEDER = 55;

    public static final double SHOOTER_MAX_RPM = 1;
    public static final double FEEDER_MAX_RPM = 1;

    public static final double SHOOTER_KP = 0;
    public static final double SHOOTER_KI = 0;
    public static final double SHOOTER_KD = 0;
    public static final double SHOOTER_KS = 0.225;
    public static final double SHOOTER_KV = 0.128;
    public static final double SHOOTER_KA = 0;

    public static final double[][] DISTANCE_RPM_MAP = {
      {1.1684, 1300},
      {2.0828, 1700},
      {2.8194, 2000},
      {3.556, 2500}
    };

    public static final double DEFAULT_RPM = 1300;
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

  public static class PoseConstants {
    public static Translation2d hubPosition() {
      final Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
        return new Translation2d(Inches.of(182.105), Inches.of(158.845));
      }
      return new Translation2d(Inches.of(469.115), Inches.of(158.845));
    }

    public static final Pose2d blueHubBasePose =
        new Pose2d(Inches.of(132), Inches.of(158.845), new Rotation2d(0));

    public static final Pose2d redHubBasePose =
        new Pose2d(Inches.of(519.221), Inches.of(158.845), new Rotation2d(0));

    public static final double AutoAlignTranslationTolerance = 0.1; // meters
    public static final double AutoAlignAngleTolerance = 5; // degrees
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
