package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        public RigidClimbers() {
                // rightRigidClimber.setInverted(false);
                // leftRigidClimber.setInverted(true);

        }
        // get motors, make group

        public boolean atSetpoint() {
                double leftCurrentPosition = leftRigidClimber.getEncoder().getPosition();
                double leftError = Math.abs(this.currentSetpoint - leftCurrentPosition);
                double rightCurrentPosition = rightRigidClimber.getEncoder().getPosition();
                double rightError = Math.abs(this.currentSetpoint - rightCurrentPosition);
                System.out.println("atSetpoint called" + this.currentSetpoint + " position "
                                + leftCurrentPosition + " "
                                + rightCurrentPosition);
                return (leftError < ClimberConstants.rigidPIDTolerance
                                && rightError < ClimberConstants.rigidPIDTolerance);
        }
        // PID4

        /*
         * public boolean isAtLegalMax(boolean isLeft) {
         * if (isLeft) {
         * return leftRigidClimber.getEncoder().getPosition() >=
         * ClimberConstants.RIGID_CLIMBER_LEGAL_MAX;
         * } else {
         * return rightRigidClimber.getEncoder().getPosition() >=
         * ClimberConstants.RIGID_CLIMBER_LEGAL_MAX;
         * }
         * }
         */

        public void setPosition(double desiredPosition) {
                this.currentSetpoint = desiredPosition;
                var leftPID = leftRigidClimber.getPIDController();
                leftPID.setOutputRange(-0.75, 0.75);
                leftPID.setReference(desiredPosition - leftMotorOffset,
                                ControlType.kPosition);
                var rightPID = rightRigidClimber.getPIDController();
                rightPID.setOutputRange(-0.75, 0.75);
                rightPID.setReference(desiredPosition - rightMotorOffset,
                                ControlType.kPosition);
        }

        public double getEncoderValue() {
                return leftRigidClimber.getEncoder().getPosition();
        }

        // setting position

        public double getLeftCurrent() {
                return leftRigidClimber.getOutputCurrent();
        }

        public double getRightCurrent() {
                return rightRigidClimber.getOutputCurrent();
        }
        // get left current

        public double getLeftSpeed() {
                return leftRigidClimber.getEncoder().getVelocity();
        }

        public double getRightSpeed() {
                return rightRigidClimber.getEncoder().getVelocity();
        }

        public void zeroLeft() {
                leftRigidClimber.getEncoder().setPosition(0);
        }

        public void zeroRight() {
                rightRigidClimber.getEncoder().setPosition(0);
        }

        public void setSpeed(double desiredSpeed) {
                rigidClimberGroup.set(desiredSpeed);
                SmartDashboard.putNumber("left rigid pos",
                                leftRigidClimber.getEncoder().getPosition());
                SmartDashboard.putNumber("right rigid pos",
                                rightRigidClimber.getEncoder().getPosition());

        }
        // set speed

        public void upHalfSpeed() {
                leftRigidClimber.set(0.5);
                rightRigidClimber.set(0.5);
        }

        public void downHalfSpeed() {
                leftRigidClimber.set(-0.5);
                rightRigidClimber.set(-0.5);
        }

        public void setSpeedLeft(double desiredSpeed) {
                leftRigidClimber.set(desiredSpeed);
        }

        public void setSpeedRight(double desiredSpeed) {
                rightRigidClimber.set(desiredSpeed);
        }
}
