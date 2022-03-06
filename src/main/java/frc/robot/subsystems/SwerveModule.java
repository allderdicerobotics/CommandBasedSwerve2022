// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.ThriftyEncoder;

public class SwerveModule {

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final ThriftyEncoder m_turningEncoder;

  // private final PIDController m_drivePIDController =
  // new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  private final SparkMaxPIDController m_drivePIDController;

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1.1, 0, 0, new TrapezoidProfile.Constraints(
      Constants.ModuleConstants.kModuleMaxAngularVelocity, Constants.ModuleConstants.kModuleMaxAngularAcceleration));

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.33217, 2.5407, 0.52052);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.25, 0.2);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorID          CAN ID for the drive motor.
   * @param turningMotorID        CAN ID for the turning motor.
   * @param turningEncoderChannel Analog channel for the steering motor. ALSO
   *                              ROTATION OFFSET
   */

  public SwerveModule(int driveMotorID, int turningMotorID, ThriftyEncoder thriftyEncoder) {
    // int[] driveEncoderPorts,
    // int[] turningEncoderPorts) {
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

    m_drivePIDController = m_driveMotor.getPIDController();
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = thriftyEncoder;

    // this.m_driveEncoder = new Encoder(driveEncoderPorts[0],
    // driveEncoderPorts[1]);

    // this.m_turningEncoder = new Encoder(turningEncoderPorts[0],
    // turningEncoderPorts[1]);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // // Set whether drive encoder should be reversed or not
    // m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // // Set the distance (in this case, angle) per pulse for the turning
    // encoder.
    // // This is the the angle through an entire rotation (2 * pi) divided by
    // the
    // // encoder resolution.
    // m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // // Set whether turning encoder should be reversed or not
    // m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the
    // input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // public SwerveModule(int kfrontleftdrivemotorport, int
  // kfrontleftturningmotorport,
  // ThriftyEncoder thriftyEncoder) {
  // }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    SmartDashboard.putNumber("Encoder Position" + this.m_turningMotor.getDeviceId(), m_turningEncoder.getPositionRadians());
    SmartDashboard.putNumber("Encoder Voltage" + this.m_turningMotor.getDeviceId(), m_turningEncoder.input.getVoltage());
    // return new SwerveModuleState(m_driveEncoder.getRate(), new
    // Rotation2d(m_turningEncoder.get()));
    return new SwerveModuleState(m_driveEncoder.getVelocity(), m_turningEncoder.getPosition());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        // SwerveModuleState.optimize(desiredState, new
        // Rotation2d(m_turningEncoder.get()));
        SwerveModuleState.optimize(desiredState, m_turningEncoder.getPosition());
    SmartDashboard.putNumber("optimized state" + this.m_turningMotor.getDeviceId(), state.angle.getRadians());

    // Calculate the drive output from the drive PID controller.
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getPositionRadians(), state.angle.getRadians());
    SmartDashboard.putNumber("turn voltage out" + this.m_turningMotor.getDeviceId(), turnOutput);

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    final double driveMotorRPM = state.speedMetersPerSecond * 60 / Math.PI / Constants.ModuleConstants.kWheelRadius / 2
        * Constants.ModuleConstants.kDriveWheelGearRatio;
    m_drivePIDController.setReference(driveMotorRPM, ControlType.kVelocity, 2, driveFeedforward, ArbFFUnits.kVoltage);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.reset();
    m_turningEncoder.reset();
  }
}
