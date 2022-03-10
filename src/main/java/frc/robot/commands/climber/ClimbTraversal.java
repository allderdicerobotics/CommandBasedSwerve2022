package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;

public class ClimbTraversal extends SequentialCommandGroup {

  public ClimbTraversal(
    RigidClimbers rigidClimbers,
    RotatingClimbers rotatingClimbers
  ) {
    addCommands(
      new ClimbersToPosition(
        rigidClimbers,
        rotatingClimbers,
        ClimberConstants.RIGID_CLIMBERS_MIN,
        ClimberConstants.ROTATING_CLIMBERS_F_SM
      ),
      // lift robot up, get rotating climbers on bar
      new ClimbersToPosition(
        rigidClimbers,
        rotatingClimbers,
        ClimberConstants.RIGID_CLIMBERS_EXTEND_ROTATING_ON_BAR,
        ClimberConstants.ROTATING_CLIMBERS_F_SM
      )
    );
    // lower robot so that rotating arms hook on bar
  }
}
