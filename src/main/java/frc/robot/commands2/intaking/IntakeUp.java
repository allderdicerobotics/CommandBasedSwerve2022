package frc.robot.commands2.intaking;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeActuation;

public class IntakeUp extends CommandBase {

    private IntakeActuation actuationSubsystem;

    public IntakeUp(IntakeActuation subsystem) {
        actuationSubsystem = subsystem;
        addRequirements(actuationSubsystem);
    }

    @Override
    public void initialize() {
        actuationSubsystem.setGoal(Constants.IntakeConstants.upPosition);
    }

    @Override

    public boolean isFinished() {
        return true;
    }
}
