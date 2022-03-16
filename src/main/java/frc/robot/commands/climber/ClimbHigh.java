package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;

// climb high
public class ClimbHigh extends SequentialCommandGroup {
    public ClimbHigh(RigidClimbers rigidClimbers, RotatingClimbers rotatingClimbers) {
        addCommands(); // TODO
    }
}
