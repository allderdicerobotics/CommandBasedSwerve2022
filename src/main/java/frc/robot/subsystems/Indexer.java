// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// get can id of indexer spark max, set the speed of the motor, define methods
public class Indexer extends SubsystemBase {

  private final CANSparkMax indexerMotor = new CANSparkMax(
    Constants.IntakeConstants.indexerMotorPort,
    MotorType.kBrushless
  );

  public void setSpeed(double speed) {
    indexerMotor.set(speed);
  }

  public void stop() {
    indexerMotor.set(0);
  }

  public void indexerInFast() {
    setSpeed(-0.5); // TODO: set speed
  }

  public void indexerInSlow() {
    setSpeed(-0.25); // TODO: set speed
  }

  public void indexerOut() {
    setSpeed(1.0); // TODO: set speed
  }
}
