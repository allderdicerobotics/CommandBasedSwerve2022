package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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
                    rigidClimbers.setPosition(rigidPosition);
                }, rigidClimbers)).withInterrupt(() -> {
                    return rigidClimbers.atSetpoint();
                }), // set climber position to 0 once

                (new RunCommand(() -> {
                    rotatingClimbers.setPosition(rotatingPosition);
                }, rotatingClimbers)).withInterrupt(() -> {
                    return rotatingClimbers.atSetpoint();
                })); // set climber position to 0 once
    }
}
