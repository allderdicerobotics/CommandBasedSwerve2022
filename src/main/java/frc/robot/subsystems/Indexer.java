// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

  private final CANSparkMax indexerMotor = new CANSparkMax(Constants.IntakeConstants.indexerMotorPort,
      MotorType.kBrushless);

  public void spinIndexer(double speed) {
    indexerMotor.set(speed);
  }

  public void stop() {
    indexerMotor.stop();
  }

  public void indexerIn() {
    spinIndexer(5.0); // check speed
  }

  public void indexerOut() {
    spinIndexer(5.0); // check speed
  }
}
