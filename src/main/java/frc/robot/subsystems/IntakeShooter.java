// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

public class IntakeShooter extends SubsystemBase {

  private final CANSparkMax intakeShooterMotor = new CANSparkMax(Constants.IntakeConstants.indexerMotorPort, MotorType.kBrushless);

  public void setSpeed(double speed) {
    intakeShooterMotor.set(speed);
  }

  public void runIn() {
    setSpeed(5.0); // TODO: find actual speed
  }

  public void runOut() {
    setSpeed(-5.0); // TODO: find actual speed
  }

}
