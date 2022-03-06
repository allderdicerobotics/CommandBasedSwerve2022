package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class IntakeActuation extends ProfiledPIDSubsystem {

  private final ArmFeedforward actuationFeedforward = new ArmFeedforward(1.41, 2.45, 0.46);

  private final CANSparkMax leadingActuationMotor = new CANSparkMax(Constants.IntakeConstants.leftActuationPort,
      MotorType.kBrushless);

  private final CANSparkMax followingActuationMotor = new CANSparkMax(Constants.IntakeConstants.rightActuationPort,
      MotorType.kBrushless);

  private RelativeEncoder actuationEncoder;

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint

    double feedforward = actuationFeedforward.calculate(setpoint.position, setpoint.velocity);

    // Add the feedforward to the PID output to get the motor output

    leadingActuationMotor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return actuationEncoder.getPosition();
  }

  public IntakeActuation() {
    super(new ProfiledPIDController(0, 0, 0,
        new TrapezoidProfile.Constraints(Constants.IntakeConstants.kMaxAngularVelocity,
            Constants.IntakeConstants.kMaxAngularAcceleration)));

    followingActuationMotor.follow(leadingActuationMotor, true);

    actuationEncoder = leadingActuationMotor.getEncoder();
    setGoal(0);
  }

  public void setPositionUp() {
    setGoal(0);
    System.out.println("intake up");
  }

  public void setPositionDown() {
    setGoal(3);
    System.out.println("intake down");
  }
}
