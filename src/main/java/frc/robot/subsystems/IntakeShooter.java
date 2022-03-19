// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

// get can id of indexer spark max, set the speed of the motor, define methods
public class IntakeShooter extends SubsystemBase {

  private final CANSparkMax intakeShooterMotor;

  public IntakeShooter() {
    this.intakeShooterMotor = new CANSparkMax(Constants.IntakeConstants.intakerShooterMotorPort,
        MotorType.kBrushless);
  }

  /*
   * public IntakeShooter(int altFrameConfig) {
   * this.intakeShooterMotor = new
   * CANSparkMax(Constants.IntakeConstants.intakerShooterMotorPort,
   * MotorType.kBrushless);
   * if (altFrameConfig > 0) {
   * this.intakeShooterMotor.setControlFramePeriodMs(altFrameConfig);
   * for (PeriodicFrame fT : PeriodicFrame.values()) {
   * this.intakeShooterMotor.setPeriodicFramePeriod(fT, altFrameConfig);
   * }
   * }
   * }
   */

  public void setSpeed(double speed) {
    intakeShooterMotor.set(speed);
  }

  public void stop() {
    setSpeed(0);
  }

  public void runIn() {
    setSpeed(0.5); // TODO: find actual speed
  }

  public void runOutSlow() {
    setSpeed(-0.5); // TODO: find actual speed
  }

  public void shootOut() {
    setSpeed(-1.0); // TODO: find actual speed
  }

}
