// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// import frc.robot.commands.LogCameraCommand;

public class Indexer extends SubsystemBase {

  private final CANSparkMax m_indexerMotor = new CANSparkMax(Constants.IntakeConstants.indexerMotorPort, MotorType.kBrushless);

  public void SpinIndexer(double speed) {
    m_indexerMotor.set(speed);
  }

  public void IndexerIn() {
    SpinIndexer(5.0); //check speed
  }

  public void IndexerOut() {
    SpinIndexer(5.0); //check speed
  }
}
