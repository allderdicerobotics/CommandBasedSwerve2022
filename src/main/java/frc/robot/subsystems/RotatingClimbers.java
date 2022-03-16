package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        // private final MotorControllerGroup rotatingClimberGroup = new
        // MotorControllerGroup(leftRotatingClimber,
        // rightRotatingClimber);

        public RotatingClimbers() {
                leftRotatingClimber.setInverted(true);
                rightRotatingClimber.setInverted(false);
        }

        public void setPosition(double desiredPosition) {
                this.currentSetpoint = desiredPosition;
                leftRotatingClimber.getPIDController().setReference(
                                desiredPosition - leftMotorOffset,
                                ControlType.kPosition);
                rightRotatingClimber.getPIDController().setReference(
                                desiredPosition - rightMotorOffset,
                                ControlType.kPosition);
        }

        public double getLeftSpeed() {
                return leftRotatingClimber.getEncoder().getVelocity();
        }

        public double getRightSpeed() {
                return rightRotatingClimber.getEncoder().getVelocity();
        }

        public void zeroLeft() {
                leftRotatingClimber.getEncoder().setPosition(0);
        }

        public void zeroRight() {
                rightRotatingClimber.getEncoder().setPosition(0);
        }

        public void setSpeed(double desiredSpeed) {
                leftRotatingClimber.set(desiredSpeed);
                rightRotatingClimber.set(desiredSpeed);
                SmartDashboard.putNumber("left rotating pos",
                                leftRotatingClimber.getEncoder().getPosition());
                SmartDashboard.putNumber("right rotating pos",
                                rightRotatingClimber.getEncoder().getPosition());
        }

        public boolean atSetpoint() {
                double leftCurrentPosition = leftRotatingClimber.getEncoder().getPosition();
                double leftError = Math.abs(this.currentSetpoint - leftCurrentPosition);
                double rightCurrentPosition = rightRotatingClimber.getEncoder().getPosition();
                double rightError = Math.abs(this.currentSetpoint - rightCurrentPosition);
                System.out.println("rotating at setpoint");
                return (leftError < ClimberConstants.rotatingPIDTolerance
                                && rightError < ClimberConstants.rotatingPIDTolerance);
        }

        public void upHalfSpeed() {
                leftRotatingClimber.set(0.5);
                rightRotatingClimber.set(0.5);
        }

        public void downHalfSpeed() {
                leftRotatingClimber.set(-0.5);
                rightRotatingClimber.set(-0.5);
        }

        public void setSpeedLeft(double desiredSpeed) {
                leftRotatingClimber.set(desiredSpeed);
        }

        public void setSpeedRight(double desiredSpeed) {
                rightRotatingClimber.set(desiredSpeed);
        }
}
