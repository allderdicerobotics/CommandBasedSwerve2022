package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;

public class ClimbMid extends SequentialCommandGroup {
        public ClimbMid(RigidClimbers rigidClimbers, RotatingClimbers rotatingClimbers) {
                addCommands(
                                // new RigidLimitSwitchZero(rigidClimbers),
                                // new RotatingLimitSwitchZero(rotatingClimbers),
                                // new HomeRigidClimbers(rigidClimbers)
                                // new ClimbersToPosition(
                                // rigidClimbers,
                                // rotatingClimbers,
                                // ClimberConstants.RIGID_CLIMBERS_MAX,
                                // ClimberConstants.ROTATING_CLIMBERS_B_SM),
                                // new WaitCommand(1),
                                new ClimbersToPosition(
                                                rigidClimbers,
                                                rotatingClimbers,
                                                ClimberConstants.RIGID_CLIMBERS_MAX, // was MIN ??
                                                ClimberConstants.ROTATING_CLIMBERS_B_SM));

                // lift rigid arms, turn rotating ones away from mid bar
                // new DriveToPosition( TODO: DRIVE FORWARDS);
                // drives robot to mid bar

        }
}
