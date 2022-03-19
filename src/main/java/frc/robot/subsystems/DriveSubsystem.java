// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.ThriftyEncoder;

import java.util.function.*;

public class DriveSubsystem extends SubsystemBase {
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  private boolean overrideFocState = false;

  // set up swerve modules
  private final SwerveModule frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      new ThriftyEncoder(new AnalogInput(0), Rotation2d.fromDegrees(30)));

  private final SwerveModule rearLeft = new SwerveModule(DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      new ThriftyEncoder(new AnalogInput(1), Rotation2d.fromDegrees(30)));

  private final SwerveModule frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      new ThriftyEncoder(new AnalogInput(2), Rotation2d.fromDegrees(90)));

  private final SwerveModule rearRight = new SwerveModule(DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      new ThriftyEncoder(new AnalogInput(3), Rotation2d.fromDegrees(30)));

  // The NavX
  // TODO: FIX NAVX CONFIGURATION NOW THAT IT'S VERTICAL
  private final AHRS navX = new AHRS(SerialPort.Port.kUSB);

  // Odometry class for tracking robot pose
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kinematics,
      this.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(this.getRotation2d(), frontLeft.getState(), frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, this.getRotation2d());
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

  public boolean evalFocOverride(boolean prior) {
    return (this.overrideFocState ? true : prior);
  }

  public void toggleFocOverride() {
    System.out.print("TOGGLED DRIVESUBSYSTEM FOC OVERRIDE");
    this.overrideFocState = !this.overrideFocState;
    System.out.printf(" -> %s\n", this.overrideFocState ? "on" : "off");
  }

  // Field oriented control
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelativeGiven) {
    boolean fieldRelative = this.evalFocOverride(fieldRelativeGiven);
    // System.out.printf("<recv FOC:%s>", fieldRelative ? false : true);
    // System.out.println("IN DRIVE");
    /*
     * System.out.printf(
     * "drive params: xSpeed=%4f, ySpeed=%f, rot=%4f\n",
     * xSpeed, ySpeed, rot);
     */
    var swerveModuleStates = DriveConstants.kinematics
        .toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, this.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);

    // SmartDashboard.putNumber("Front Left Speed",
    // frontLeft.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("Front Left Angle",
    // frontLeft.getState().angle.getRadians());
    // SmartDashboard.putNumber("Front Left Desired Angle",
    // swerveModuleStates[0].angle.getRadians());
  }

  // drive accorsing to joystick
  public void driveWithJoystick(double leftX, double leftY, double rightX, boolean fieldRelative) {
    // Get the x speed. We are inverting this because Playstation controllers return
    // negative values when we push forward.
    final var xSpeed = xspeedLimiter.calculate(MathUtil.applyDeadband(leftX, 0.1)) * DriveConstants.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left.
    final var ySpeed = -yspeedLimiter.calculate(MathUtil.applyDeadband(leftY, 0.1)) * DriveConstants.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics).
    final var rot = -rotLimiter.calculate(MathUtil.applyDeadband(rightX, 0.05))
        * DriveConstants.kMaxAngularSpeed;

    // SmartDashboard.putNumber("Joystick X Speed", leftX);
    // SmartDashboard.putNumber("Joystick Y Speed", leftY);
    // SmartDashboard.putNumber("Joystick Rotation", rightX);

    // System.out.printf("x=%s, y=%s, rot=%s, fr=%s", xSpeed, ySpeed, rot,
    // fieldRelative);

    // System.out.println("IN DRIVEWITHJOYSTICK");

    // SmartDashboard.putNumber("Joystick thing", 2);

    /*
     * SmartDashboard.putString("Joystick data", String.format(
     * "xs=%4d, ys=%4d, rots=%4d, fr=%s",
     * xSpeed, ySpeed, rot, (fieldRelative ? "t" : "f")));
     */

    this.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  // drive accorsing to joystick
  public void driveWithJoystick(double leftX, double leftY, double rightX, Supplier<Boolean> fieldRelativeFn) {
    // Get the x speed. We are inverting this because Playstation controllers return
    // negative values when we push forward.
    final var xSpeed = xspeedLimiter.calculate(MathUtil.applyDeadband(leftX, 0.1)) * DriveConstants.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left.
    final var ySpeed = -yspeedLimiter.calculate(MathUtil.applyDeadband(leftY, 0.1)) * DriveConstants.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics).
    final var rot = -rotLimiter.calculate(MathUtil.applyDeadband(rightX, 0.05))
        * DriveConstants.kMaxAngularSpeed;

    // SmartDashboard.putNumber("Joystick X Speed", leftX);
    // SmartDashboard.putNumber("Joystick Y Speed", leftY);
    // SmartDashboard.putNumber("Joystick Rotation", rightX);

    this.drive(xSpeed, ySpeed, rot, fieldRelativeFn.get());
  }

  public void stop() {
    this.drive(0.0, 0.0, 0.0, false);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    // TODO
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  public Rotation2d getRotation2d() {
    /*
     * if (this.overrideFocState) {
     * return new Rotation2d((navX.getRoll() / 360) * 2 * Math.PI);
     * }
     */
    return navX.getRotation2d();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navX.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return this.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navX.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
