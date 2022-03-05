// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

public class IntakerRod extends SubsystemBase {

  private final CANSparkMax m_intakerRodMotor = new CANSparkMax(Constants.IntakeConstants.indexerMotorPort, MotorType.kBrushless);

  public void SpinIntaker(double speed) {
    m_intakerRodMotor.set(speed);
  }

  public void SpinIntakerIn() {
    SpinIntaker(5.0); //check speed
  }

  public void SpinIntakerOut() {
    SpinIntaker(-5.0); //check speed
  }

}
