// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    public static final int kFrontLeftDriveMotorPort = 10;
    public static final int kRearLeftDriveMotorPort = 11;
    public static final int kFrontRightDriveMotorPort = 12;
    public static final int kRearRightDriveMotorPort = 13;

    public static final int kFrontLeftTurningMotorPort = 14;
    public static final int kRearLeftTurningMotorPort = 15;
    public static final int kFrontRightTurningMotorPort = 16;
    public static final int kRearRightTurningMotorPort = 17;
    public static final boolean kGyroReversed = false;

    public static final Translation2d frontLeftLocation = new Translation2d(-0.381, 0.381); // TODO
    public static final Translation2d frontRightLocation = new Translation2d(0.381, 0.381);
    public static final Translation2d backLeftLocation = new Translation2d(-0.381, -0.381);
    public static final Translation2d backRightLocation = new Translation2d(0.381, -0.381);

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation);
  }

  public static final class IntakeConstants {

    public static final int indexerMotorPort = 4;
    public static final int intakerShooterMotorPort = 5;

  }

  public static final class ActuationConstants {
    public static final int rightMotorPort = 2;
    public static final int leftMotorPort = 3;

    public static final double ks = 0.0;
    public static final double kcos = 0.5;
    public static final double kv = 1.0;
    public static final double kp = 4.0;
    public static final double ki = 0.0;
    public static final double kd = 0.0;

    public static final double kMaxAngularVelocity = 2;
    public static final double kMaxAngularAcceleration = 2;

    public static final double downPosition = 0.0;
    public static final double upPosition = 1.2;
  }

  public static final class ClimberConstants { // rotates ~0-40
    public static final int rightRigidClimberPort = 8;
    public static final int leftRigidClimberPort = 9;
    public static final int rightRotatingClimberPort = 6;
    public static final int leftRotatingClimberPort = 7;

    // public static final int leftRigidLimitSwitchPort = 5;
    // public static final int rightRigidLimitSwitchPort = 7;
    // public static final int leftRotatingLimitSwitchPort = 8;
    // public static final int rightRotatingLimitSwitchPort = 9;

    public static final double rigidPIDTolerance = 10.0;
    public static final double rotatingPIDTolerance = 5;

    public static final double maxRigidSetpoint = 10;
    public static final double minRigidSetpoint = 0;

    public static final double RIGID_CLIMBERS_MAX = 229; // TODO: get these
    public static final double RIGID_CLIMBERS_MIN = -5;
    public static final double RIGID_CLIMBERS_EXTEND_ROTATING_ON_BAR = 10;
    public static final double RIGID_CLIMBERS_EXTEND_ROTATING_ON_BAR_SECOND = 20;
    public static final double RIGID_CLIMBERS_RELEASE_BAR = 50;
    public static final double RIGID_CLIMBERS_EXTEND_ROTATING_UNDER_BAR = 40;
    public static final double RIGID_CLIMBERS_HALF = 110;

    public static final double ROTATING_CLIMBERS_ON_BAR = 18;
    public static final double ROTATING_CLIMBERS_RELEASE_BAR = 35;
    public static final double ROTATING_CLIMBERS_PAST_BAR = 61;
    public static final double ROTATING_CLIMBERS_GRAB_BAR = 40;

    public static final double ROTATING_CLIMBERS_F_SM = 10;
    public static final double ROTATING_CLIMBERS_B_SM = 0; // TODO: can be bigger so that rotating
                                                           // arms don't have to swing as far
    public static final double ROTATING_CLIMBERS_F_L = 9;
    public static final double ROTATING_CLIMBERS_B_ROTATING_ON_BAR = 7;
    public static final double ROTATING_CLIMBERS_B_ROTATING_ON_BAR_SECOND = 15;
    public static final double ROTATING_CLIMBERS_B_ROTATING_OFF_BAR = 3;

    public static final double rigidHomeCurrent = 10;
    public static final double rigidHomeSpeed = 0.4;
    public static final double rotatingHomeCurrent = 1;
  }

  public static final class ModuleConstants {
    public static final double kWheelRadius = 0.0508;
    public static final double kDriveWheelGearRatio = 6.55;

    public static final double kModuleMaxAngularVelocity = 4 * 2 * Math.PI;
    public static final double kModuleMaxAngularAcceleration = 4 * 2 * Math.PI; // radians per
                                                                                // second squared
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class VisionConstants {
    public static final double CAMERA_HEIGHT_METERS = 1;
    public static final double TARGET_HEIGHT_METERS = 2;
    public static final double CAMERA_PITCH_RADIANS = 0;
    public static final double GOAL_RANGE_METERS = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
