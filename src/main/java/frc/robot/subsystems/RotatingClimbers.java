package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class RotatingClimbers extends SubsystemBase {
        public double currentSetpoint;
        public double leftMotorOffset = 0;
        public double rightMotorOffset = 0;

        private final CANSparkMax leftRotatingClimber = new CANSparkMax(
                        Constants.ClimberConstants.leftRotatingClimberPort, MotorType.kBrushless);
        private final CANSparkMax rightRotatingClimber = new CANSparkMax(
                        Constants.ClimberConstants.rightRotatingClimberPort, MotorType.kBrushless);

        private final MotorControllerGroup rotatingClimberGroup = new MotorControllerGroup(leftRotatingClimber,
                        rightRotatingClimber);

        public void setPosition(double desiredPosition) {
                this.currentSetpoint = desiredPosition;
                leftRotatingClimber.getPIDController().setReference(desiredPosition - leftMotorOffset,
                                ControlType.kPosition);
                rightRotatingClimber.getPIDController().setReference(desiredPosition - rightMotorOffset,
                                ControlType.kPosition);
        }

        public void setSpeed(double desiredSpeed) {
                rotatingClimberGroup.set(desiredSpeed);
        }

        public boolean atSetpoint() {
                double leftCurrentPosition = leftRotatingClimber.getEncoder().getPosition();
                double leftError = Math.abs(this.currentSetpoint - leftCurrentPosition);
                double rightCurrentPosition = rightRotatingClimber.getEncoder().getPosition();
                double rightError = Math.abs(this.currentSetpoint - rightCurrentPosition);
                return (leftError < ClimberConstants.rotatingPIDTolerance
                                && rightError < ClimberConstants.rotatingPIDTolerance);
        }
}