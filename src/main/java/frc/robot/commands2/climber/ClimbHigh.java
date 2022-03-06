package frc.robot.commands2.climber;

public class ClimbHigh extends SequentialCommandGroup {
	public ClimbHigh(ClimberControl climber) {
		addCommands(
			new ClimbMid(control),
			new ClimbStep(control),
		);
	}
}

