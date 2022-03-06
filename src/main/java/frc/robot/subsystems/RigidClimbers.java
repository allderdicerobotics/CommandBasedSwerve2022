package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RigidClimbers extends SubsystemBase {
    private final CANSparkMax leftRigidClimber = new CANSparkMax(Constants.ClimberConstants.leftRigidClimberPort,
            MotorType.kBrushless);
    private final CANSparkMax rightRigidClimber = new CANSparkMax(Constants.ClimberConstants.rightRigidClimberPort,
            MotorType.kBrushless);

    private final MotorControllerGroup rigidClimberGroup = new MotorControllerGroup(leftRigidClimber,
            rightRigidClimber);

    // public void SetPosition(double position) {

    // }

    public void setSpeed(double desiredSpeed) {
        rigidClimberGroup.set(desiredSpeed); // TODO: change to position
    }

    public void setPositon(double desiredPositon) {
        // TODO: PID set reference
    }

    public void setPositionHighest() {
        setPositon(0.0); // TODO: whatever the highest position will be
    }

    public void setPositionLowest() {
        setPositon(5.0); // TODO: whatever the highest position will be
    }
}
