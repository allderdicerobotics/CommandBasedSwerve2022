package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;

public class ClimbMid extends SequentialCommandGroup {
        public ClimbMid(RigidClimbers rigidClimbers, RotatingClimbers rotatingClimbers) {
                addCommands(
                                new ClimbersToPosition(
                                                rigidClimbers,
                                                rotatingClimbers,
                                                ClimberConstants.RIGID_CLIMBERS_UP,
                                                ClimberConstants.ROTATING_CLIMBERS_FORWARD),
                                new ClimbersToPosition(
                                                rigidClimbers,
                                                rotatingClimbers,
                                                ClimberConstants.RIGID_CLIMBERS_DOWN,
                                                ClimberConstants.ROTATING_CLIMBERS_BACKWARD));
        }
}