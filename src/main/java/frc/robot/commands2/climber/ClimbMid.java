package frc.robot.commands2.climber;

import frc.robot.Constants.ClimberConstants;
import frc.robot.commands2.helpers.DriveHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class ClimbMid extends SequentialCommandGroup {
	public ClimbMid(ClimberControl climber, DriveSubsystem drive) {
		Rotation2D forward, backward, neutral; // TODO
		addCommands(
				climber.to(new ClimberPos(ClimberConstants.RIGID_MAX, ClimberConstants.R_FORWARD)),
				DriveHelpers.timedBackwards(drive, ClimberConstants.DRIVE_BACK_TIME_SECS), // TODO
				climber.to(new ClimberPos(ClimberConstants.RIGID_MIN, ClimberConstants.R_BACKWARD)),
				climber.to(new ClimberPos(ClimberConstants.RIGID_STAGE_FOUR, ClimberConstants.R_BACKWARD),
				climber.to(new ClimberPos(ClimberConstants.RIGID_STAGE_FOUR,ClimberConstants.R_BACK_STAGE_FIVE)),
				climber.to(new ClimberPos(ClimberConstants.RIGID_MAX,ClimberConstants.R_BACK_STAGE_FIVE)),
				climber.to(new ClimberPos(ClimberConstants.RIGID_MAX,ClimberConstants.R_FORWARD_STAGE_SEVEN)),
				climber.to(new ClimberPos(ClimberConstants.RIGID_STAGE_FOUR,ClimberConstants.R_))
		// TODO
		);
	}
}
