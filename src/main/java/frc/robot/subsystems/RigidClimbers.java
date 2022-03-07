package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RigidClimbers extends SubsystemBase {
    private final CANSparkMax m_leftRigidClimber = new CANSparkMax(Constants.ClimberConstants.leftRigidClimberPort,
            MotorType.kBrushless);
    private final CANSparkMax m_rightRigidClimber = new CANSparkMax(Constants.ClimberConstants.rightRigidClimberPort,
            MotorType.kBrushless);

    private final MotorControllerGroup m_rigidClimberGroup = new MotorControllerGroup(m_leftRigidClimber,
            m_rightRigidClimber);

    private final double MIN_SETPOINT = null; // TODO
    private final double MAX_SETPOINT = null; // TODO

    public boolean closeToPosition(double position) {
        double setp = this.posToSetpoint(position);
	// TODO check `setp`
    }

    public void setPosition(double position) {
        double setp = this.posToSetpoint(position);
	// TODO make this work with `setp`
    }

    public double posToSetpoint(double position) {
        return (new MapValues())
		.from(0.0, 1.0)
		.to(MIN_SETPOINT, MAX_SETPOINT)
		.apply(position);
    }

    // public void setSpeed(double desiredSpeed) {
    //     m_rigidClimberGroup.set(desiredSpeed); // xTODO: change to position
    // }
}
