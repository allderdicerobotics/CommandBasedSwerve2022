package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;

public class StopClimbers extends SequentialCommandGroup {
    public StopClimbers(RigidClimbers rigidClimbers, RotatingClimbers rotatingClimbers) {
        rigidClimbers.setSpeed(0);
        rotatingClimbers.setSpeed(0);
    }
}
