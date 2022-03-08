package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class RigidClimbers extends SubsystemBase {
    private final CANSparkMax leftRigidClimber = new CANSparkMax(Constants.ClimberConstants.leftRigidClimberPort,
            MotorType.kBrushless);
    private final CANSparkMax rightRigidClimber = new CANSparkMax(Constants.ClimberConstants.rightRigidClimberPort,
            MotorType.kBrushless);

    private final MotorControllerGroup rigidClimberGroup = new MotorControllerGroup(leftRigidClimber,
            rightRigidClimber);

    // private final double MIN_SETPOINT = null; // TODO
    // private final double MAX_SETPOINT = null; // TODO

    // public boolean closeToPosition(double position) {
    // double setp = this.posToSetpoint(position);
    // return (
    // this.motorCloseToSetpoint(setp, this.leftRigidClimber)
    // &&
    // this.motorCloseToSetpoint(setp, this.rightRigidClimber)
    // );
    // }

    // public boolean motorCloseToSetpoint(double setp, CANSparkMax m) {
    // double curr = m.getEncoder().getPosition();
    // double rawError = Math.abs(setp - curr);
    // return rawError < ClimberConstants.RIGID_CLOSE_ENOUGH;
    // }

    // public void setHeight(double position) {
    // double setp = this.posToSetpoint(position);
    // TODO make this work with `setp`
}

// public double posToSetpoint(double position) {
// return (new MapValues())
// .from(0.0, 1.0)
// .to(MIN_SETPOINT, MAX_SETPOINT)
// .apply(position);
// }

// public void setSpeed(double desiredSpeed) {
// m_rigidClimberGroup.set(desiredSpeed); // xTODO: change to position
// }
