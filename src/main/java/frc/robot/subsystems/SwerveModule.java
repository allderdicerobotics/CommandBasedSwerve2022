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

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final ThriftyEncoder turningEncoder;

  private final SparkMaxPIDController drivePIDController;

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
    1.1,
    0,
    0,
    new TrapezoidProfile.Constraints(
      Constants.ModuleConstants.kModuleMaxAngularVelocity,
      Constants.ModuleConstants.kModuleMaxAngularAcceleration
    )
  );

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(
    0.33217,
    2.5407,
    0.52052
  );
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(
    0.25,
    0.2
  );

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorID          CAN ID for the drive motor.
   * @param turningMotorID        CAN ID for the turning motor.
   * @param turningEncoderChannel Analog channel for the steering motor. ALSO
   *                              ROTATION OFFSET
   */

  public SwerveModule(
    int driveMotorID,
    int turningMotorID,
    ThriftyEncoder thriftyEncoder
  ) {
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

    drivePIDController = driveMotor.getPIDController();
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = thriftyEncoder;

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      driveEncoder.getVelocity(),
      turningEncoder.getPosition()
    );
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
      SwerveModuleState.optimize(desiredState, turningEncoder.getPosition());
    SmartDashboard.putNumber(
      "optimized state" + this.turningMotor.getDeviceId(),
      state.angle.getRadians()
    );

    // Calculate the drive output from the drive PID controller.
    final double driveFeedforwardOut = m_driveFeedforward.calculate(
      state.speedMetersPerSecond
    );

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = turningPIDController.calculate(
      turningEncoder.getPositionRadians(),
      state.angle.getRadians()
    );
    SmartDashboard.putNumber(
      "turn voltage out" + this.turningMotor.getDeviceId(),
      turnOutput
    );

    final double turnFeedforwardOut = m_turnFeedforward.calculate(
      turningPIDController.getSetpoint().velocity
    );
    final double driveMotorRPM =
      state.speedMetersPerSecond *
      60 /
      Math.PI /
      Constants.ModuleConstants.kWheelRadius /
      2 *
      Constants.ModuleConstants.kDriveWheelGearRatio;
    drivePIDController.setReference(
      driveMotorRPM,
      ControlType.kVelocity,
      2,
      driveFeedforwardOut,
      ArbFFUnits.kVoltage
    );
    turningMotor.setVoltage(turnOutput + turnFeedforwardOut);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.resetPosition();
  }
}
