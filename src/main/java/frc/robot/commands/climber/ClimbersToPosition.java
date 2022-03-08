package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;

public class ClimbersToPosition extends ParallelCommandGroup {
    public ClimbersToPosition(
            RigidClimbers rigidClimbers,
            RotatingClimbers rotatingClimbers,
            double rigidPosition,
            double rotatingPosition) {
        addCommands(
                (new RunCommand(() -> {
                    rigidClimbers.setPosition(0);
                }, rigidClimbers)).withInterrupt(() -> {
                    return rigidClimbers.atSetpoint();
                }),

                (new RunCommand(() -> {
                    rotatingClimbers.setPosition(0);
                }, rotatingClimbers)).withInterrupt(() -> {
                    return rotatingClimbers.atSetpoint();
                }));
    }
}