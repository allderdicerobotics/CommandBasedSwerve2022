package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RotatingClimbers extends SubsystemBase {
    private final CANSparkMax m_leftRotatingClimber = new CANSparkMax(
            Constants.ClimberConstants.leftRotatingClimberPort, MotorType.kBrushless);
    private final CANSparkMax m_rightRotatingClimber = new CANSparkMax(
            Constants.ClimberConstants.rightRotatingClimberPort, MotorType.kBrushless);

    private final MotorControllerGroup m_rotatingClimberGroup = new MotorControllerGroup(m_leftRotatingClimber,
            m_rightRotatingClimber);

    // public void SetPosition(double position) {

    // }

    public void setSpeed(double desiredSpeed) {
        m_rotatingClimberGroup.set(desiredSpeed); // TODO: change to position
    }
}
