package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ActuationConstants;

public class IntakeActuation extends ProfiledPIDSubsystem {

  private final ArmFeedforward actuationFeedforward = new ArmFeedforward(ActuationConstants.ks, ActuationConstants.kcos,
      ActuationConstants.kv);

  private final CANSparkMax leadingActuationMotor = new CANSparkMax(ActuationConstants.rightMotorPort,
      MotorType.kBrushless);

  private final CANSparkMax followingActuationMotor = new CANSparkMax(ActuationConstants.leftMotorPort,
      MotorType.kBrushless);

  private DutyCycleEncoder actuationEncoder;

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
    SmartDashboard.putNumber("Intake Actuation Position", actuationEncoder.getDistance() + 5.4);
    return actuationEncoder.getDistance() + 5.4;
  }

  public IntakeActuation() {
    super(new ProfiledPIDController(ActuationConstants.kp, ActuationConstants.ki,
        ActuationConstants.kd,
        new TrapezoidProfile.Constraints(ActuationConstants.kMaxAngularVelocity,
            ActuationConstants.kMaxAngularAcceleration)));

    final ProfiledPIDController pidController = getController();
    pidController.setTolerance(0.025);
    pidController.setIntegratorRange(-2.0, 2.0);
    leadingActuationMotor.setInverted(false);
    actuationEncoder = new DutyCycleEncoder(0);
    actuationEncoder.setDistancePerRotation(-2 * Math.PI);

    // actuationEncoder.setInverted(true);
    followingActuationMotor.follow(leadingActuationMotor, true);
    enable();
  }

  public void setSpeed(double desiredSpeed) {
    setSpeed(desiredSpeed);
  }

  public void setPosition(double desiredPosition) {
    setGoal(desiredPosition);
  }

  public void setPositionUp() {
    setPosition(ActuationConstants.upPosition);
    System.out.println("intake up");
  }

  public void setPositionDown() {
    setPosition(ActuationConstants.downPosition);
    System.out.println("intake down");
  }
}
