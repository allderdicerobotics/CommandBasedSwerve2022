package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ActuationConstants;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public class ActuateWOneMotor extends SubsystemBase {
    private final CANSparkMax leftSparkMax = new CANSparkMax(ActuationConstants.leftMotorPort,
            MotorType.kBrushless);

    public double currentSetpoint;
    public double leftMotorOffset = 0;

    public void setBrake() {
        leftSparkMax.setIdleMode(IdleMode.kBrake);
    }

    public void setSpeed(double desiredSpeed) {
        leftSparkMax.set(desiredSpeed);
    }

    public void setPosition(double desiredPosition) {
        this.currentSetpoint = desiredPosition;
        var leftPID = leftSparkMax.getPIDController();
        leftPID.setOutputRange(-1.00, 1.00);
        leftPID.setReference(desiredPosition - leftMotorOffset,
                ControlType.kPosition);
    }
}
