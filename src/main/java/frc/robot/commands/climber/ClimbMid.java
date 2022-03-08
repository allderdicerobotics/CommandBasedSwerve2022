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
                                                ClimberConstants.RIGID_CLIMBERS_MAX,
                                                ClimberConstants.ROTATING_CLIMBERS_B_SM));
                // lift rigid arms, turn rotating ones away from mid bar
                // new DriveToPosition( TODO: DRIVE FORWARDS);
                // drives robot to mid bar

        }
}