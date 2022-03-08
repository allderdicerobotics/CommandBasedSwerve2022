package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;

public class ClimbTraversal extends SequentialCommandGroup {
    public ClimbTraversal(RigidClimbers rigidClimbers, RotatingClimbers rotatingClimbers) {
        addCommands();
    }
}
