package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeActuation extends SubsystemBase {

  private RelativeEncoder m_actuationEncoder;
 

  private final ProfiledPIDController m_actuationPidController =
  new ProfiledPIDController(
      2.0,
      0,
      0,
      new TrapezoidProfile.Constraints(
          Constants.IntakeConstants.kMaxAngularVelocity, Constants.IntakeConstants.kMaxAngularAcceleration));

private final ArmFeedforward m_actuationFeedforward = new ArmFeedforward(0.01, 0.0001, 0, 0);

  private double currentSetpoint = 0;

  private final CANSparkMax LeftActuationMotor = new CANSparkMax(
    Constants.IntakeConstants.leftActuationPort,
    MotorType.kBrushless
  );

  private final CANSparkMax RightActuationMotor = new CANSparkMax(
    Constants.IntakeConstants.rightActuationPort,
    MotorType.kBrushless
  );

  public IntakeActuation() {
    RightActuationMotor.follow(LeftActuationMotor, true);

    //m_actuationPID = LeftActuationMotor.getPIDController();
    m_actuationEncoder = LeftActuationMotor.getEncoder();
  }


  @Override
  public void periodic() {
    final double pidOutput = m_actuationPidController.calculate(m_actuationEncoder.getPosition(), currentSetpoint);
    final double ffOutput = m_actuationFeedforward.calculate(currentSetpoint, 1);
    System.out.println(pidOutput + ffOutput);
    LeftActuationMotor.setVoltage(pidOutput + ffOutput);
  }

  public void setPosition(double desiredPosition) {
    this.currentSetpoint = desiredPosition;
  }

  public void SetPositionUp() {
    setPosition(0);
    System.out.println("intake up");
  }

  public void SetPositionDown() {
    setPosition(3);
    System.out.println("intake down");
  }
}
