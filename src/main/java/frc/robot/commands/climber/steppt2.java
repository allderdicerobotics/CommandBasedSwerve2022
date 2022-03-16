package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;

public class steppt2 extends SequentialCommandGroup {
        public steppt2(RigidClimbers rigidClimbers, RotatingClimbers rotatingClimbers) {
                addCommands(
                                new ClimbersToPosition(
                                                rigidClimbers,
                                                rotatingClimbers,
                                                ClimberConstants.RIGID_CLIMBERS_EXTEND_ROTATING_ON_BAR,
                                                ClimberConstants.ROTATING_CLIMBERS_ON_BAR),
                                new ClimbersToPosition(
                                                rigidClimbers,
                                                rotatingClimbers,
                                                ClimberConstants.RIGID_CLIMBERS_RELEASE_BAR,
                                                ClimberConstants.ROTATING_CLIMBERS_ON_BAR),
                                new WaitCommand(3),
                                new ClimbersToPosition(
                                                rigidClimbers,
                                                rotatingClimbers,
                                                ClimberConstants.RIGID_CLIMBERS_RELEASE_BAR,
                                                ClimberConstants.ROTATING_CLIMBERS_PAST_BAR));
        }
}
