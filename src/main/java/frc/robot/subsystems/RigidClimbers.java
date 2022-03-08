package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class RigidClimbers extends SubsystemBase {

        public double currentSetpoint;
        public double leftMotorOffset = 0;
        public double rightMotorOffset = 0;

        private final CANSparkMax leftRigidClimber = new CANSparkMax(Constants.ClimberConstants.leftRigidClimberPort,
                        MotorType.kBrushless);
        private final CANSparkMax rightRigidClimber = new CANSparkMax(Constants.ClimberConstants.rightRigidClimberPort,
                        MotorType.kBrushless);

        private final MotorControllerGroup rigidClimberGroup = new MotorControllerGroup(leftRigidClimber,
                        rightRigidClimber);

        // get motors, make group

        public boolean atSetpoint() {
                double leftCurrentPosition = leftRigidClimber.getEncoder().getPosition();
                double leftError = Math.abs(this.currentSetpoint - leftCurrentPosition);
                double rightCurrentPosition = rightRigidClimber.getEncoder().getPosition();
                double rightError = Math.abs(this.currentSetpoint - rightCurrentPosition);
                return (leftError < ClimberConstants.rigidPIDTolerance
                                && rightError < ClimberConstants.rigidPIDTolerance);
        }
        // PID

        public void setPosition(double desiredPosition) {
                this.currentSetpoint = desiredPosition;
                leftRigidClimber.getPIDController().setReference(desiredPosition - leftMotorOffset,
                                ControlType.kPosition);
                rightRigidClimber.getPIDController().setReference(desiredPosition - rightMotorOffset,
                                ControlType.kPosition);
        }
        // setting position

        public double getLeftCurrent() {
                return leftRigidClimber.getOutputCurrent();
        }
        // get left current

        public void setSpeed(double desiredSpeed) {
                rigidClimberGroup.set(desiredSpeed);
        }
        // set speed
}