// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.ThriftyEncoder;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules

  private final Translation2d m_frontLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, -0.381);
  private final Translation2d m_backRightLocation = new Translation2d(0.381, -0.381);

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort, new ThriftyEncoder(new AnalogInput(0), Rotation2d.fromDegrees(30)));

  private final SwerveModule m_rearLeft = new SwerveModule(DriveConstants.kRearLeftDriveMotorPort, DriveConstants.kRearLeftTurningMotorPort,
      new ThriftyEncoder(new AnalogInput(1), Rotation2d.fromDegrees(30)));

  private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort, new ThriftyEncoder(new AnalogInput(2), Rotation2d.fromDegrees(90)));

  private final SwerveModule m_rearRight = new SwerveModule(DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort, new ThriftyEncoder(new AnalogInput(3), Rotation2d.fromDegrees(30)));

  // The gyro sensor
  // TODO: FIX NAVX CONFIGURATION NOW THAT IT'S VERTICAL
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);
  // Odometry class for tracking robot pose
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
      m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

  // SwerveDriveOdometry m_odometry =
  // new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
  // m_gyro.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */

  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics
        .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    SmartDashboard.putNumber("Front Left Speed", m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Left Angle", m_frontLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("Front Left Desired Angle", swerveModuleStates[0].angle.getRadians());

    m_frontRight.setDesiredState(swerveModuleStates[1]);
    SmartDashboard.putNumber("Front Right Speed", m_frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right Angle", m_frontRight.getState().angle.getRadians());
    SmartDashboard.putNumber("Front Right Desired Angle", swerveModuleStates[1].angle.getRadians());
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    SmartDashboard.putNumber("Back Left Speed", m_rearLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left Angle", m_rearLeft.getState().angle.getRadians());
    SmartDashboard.putNumber("Back Left Desired Angle", swerveModuleStates[2].angle.getRadians());
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    SmartDashboard.putNumber("Back Right Speed", m_rearRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right Angle", m_rearRight.getState().angle.getRadians());
    SmartDashboard.putNumber("Back Right Desired Angle", swerveModuleStates[3].angle.getRadians());
  }

  public void driveWithJoystick(double leftX, double leftY, double rightX, boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband(leftX, 0.05)) * DriveConstants.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(leftY, 0.05)) * DriveConstants.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(rightX, 0.05)) * DriveConstants.kMaxAngularSpeed;

    SmartDashboard.putNumber("Joystick X Speed", leftX);
    SmartDashboard.putNumber("Joystick Y Speed", leftY);
    SmartDashboard.putNumber("Joystick Rotation", rightX);
    this.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
  // /** Resets the drive encoders to currently read a position of 0. */
  // public void resetEncoders() {
  // m_frontLeft.resetEncoders();
  // m_rearLeft.resetEncoders();
  // m_frontRight.resetEncoders();
  // m_rearRight.resetEncoders();
  // }

  // /** Zeroes the heading of the robot. */
  // public void zeroHeading() {
  // m_gyro.reset();
  // }

  // /**
  // * Returns the heading of the robot.
  // *
  // * @return the robot's heading in degrees, from -180 to 180
  // */
  // public double getHeading() {
  // return m_gyro.getRotation2d().getDegrees();
  // }

  // /**
  // * Returns the turn rate of the robot.
  // *
  // * @return The turn rate of the robot, in degrees per second
  // */
  // public double getTurnRate() {
  // return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }
}
