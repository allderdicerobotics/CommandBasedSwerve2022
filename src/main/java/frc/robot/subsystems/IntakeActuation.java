package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class IntakeActuation extends ProfiledPIDSubsystem {

  private final ArmFeedforward actuationFeedforward = new ArmFeedforward(1.0, 2.0, 0.46);

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
    // System.out.println("voltage set");
    leadingActuationMotor.setVoltage(output + feedforward);
    SmartDashboard.putNumber("Intake Actuation PID Output", output);
    SmartDashboard.putNumber("Intake Actuation FF Output", feedforward);
  }

  @Override
  public double getMeasurement() {
    SmartDashboard.putNumber("Intake Actuation Position", actuationEncoder.getPosition());
    // SmartDashboard.putNumber("Intake Actuation Setpoint", );
    return actuationEncoder.getPosition();
  }

  public IntakeActuation() {
    super(new ProfiledPIDController(0.0001, 0, 0,
        new TrapezoidProfile.Constraints(Constants.IntakeConstants.kMaxAngularVelocity,
            Constants.IntakeConstants.kMaxAngularAcceleration)));
    leadingActuationMotor.setInverted(true);
    actuationEncoder = leadingActuationMotor.getEncoder();
    actuationEncoder.setPositionConversionFactor(2 * Math.PI / 20);
    actuationEncoder.setVelocityConversionFactor(2 * Math.PI / 20);
    followingActuationMotor.follow(leadingActuationMotor, true);

    // setGoal(0);
    enable();
  }

  public void setPositionUp() {
    setGoal(1.5);
    System.out.println("intake up");
  }

  public void setPositionDown() {
    setGoal(0);
    System.out.println("intake down");
  }
}
