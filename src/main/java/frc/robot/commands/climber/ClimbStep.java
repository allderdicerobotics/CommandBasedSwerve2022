package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;

public class ClimbStep extends SequentialCommandGroup {
        public ClimbStep(RigidClimbers rigidClimbers, RotatingClimbers rotatingClimbers) {
                addCommands(
                                new ClimbersToPosition(
                                                rigidClimbers,
                                                rotatingClimbers,
                                                ClimberConstants.RIGID_CLIMBERS_MIN,
                                                ClimberConstants.ROTATING_CLIMBERS_ON_BAR),
                                // lift robot up, get rotating climbers on bar
                                new ClimbersToPosition(
                                                rigidClimbers,
                                                rotatingClimbers,
                                                ClimberConstants.RIGID_CLIMBERS_EXTEND_ROTATING_ON_BAR,
                                                ClimberConstants.ROTATING_CLIMBERS_RELEASE_BAR),
                                // lower robot so that rotating arms hook on bar
                                new ClimbersToPosition(
                                                rigidClimbers,
                                                rotatingClimbers,
                                                ClimberConstants.RIGID_CLIMBERS_RELEASE_BAR,
                                                ClimberConstants.ROTATING_CLIMBERS_RELEASE_BAR),
                                // raise rigid climbers half way
                                new ClimbersToPosition(
                                                rigidClimbers,
                                                rotatingClimbers,
                                                ClimberConstants.RIGID_CLIMBERS_MAX,
                                                ClimberConstants.ROTATING_CLIMBERS_PAST_BAR),
                                // turn robot a lot so that rigid arms are past next bar
                                new ClimbersToPosition(
                                                rigidClimbers,
                                                rotatingClimbers,
                                                ClimberConstants.RIGID_CLIMBERS_MAX,
                                                ClimberConstants.ROTATING_CLIMBERS_GRAB_BAR)
                // extend rigid arms to max
                // new ClimbersToPosition(
                // rigidClimbers,
                // rotatingClimbers,
                // ClimberConstants.RIGID_CLIMBERS_EXTEND_ROTATING_UNDER_BAR,
                // ClimberConstants.ROTATING_CLIMBERS_GRAB_BAR)
                // // turn robot so that rigid arms can hook onto next bar
                // new ClimbersToPosition(
                // rigidClimbers,
                // rotatingClimbers,
                // ClimberConstants.RIGID_CLIMBERS_MIN,
                // ClimberConstants.ROTATING_CLIMBERS_B_ROTATING_ON_BAR),
                // // lift robot to next bar
                // new ClimbersToPosition(
                // rigidClimbers,
                // rotatingClimbers,
                // ClimberConstants.RIGID_CLIMBERS_MIN,
                // ClimberConstants.ROTATING_CLIMBERS_B_ROTATING_OFF_BAR),
                // // get rotating arms off old bar
                // new ClimbersToPosition(
                // rigidClimbers,
                // rotatingClimbers,
                // ClimberConstants.RIGID_CLIMBERS_EXTEND_ROTATING_UNDER_BAR,
                // ClimberConstants.ROTATING_CLIMBERS_B_ROTATING_OFF_BAR),
                // // lower robot so that rotating arms can fit under new bar
                // new ClimbersToPosition(
                // rigidClimbers,
                // rotatingClimbers,
                // ClimberConstants.RIGID_CLIMBERS_EXTEND_ROTATING_UNDER_BAR,
                // ClimberConstants.ROTATING_CLIMBERS_B_SM));
                );
                // turn rotating arms so that they move to other side of the bar
        }
}
