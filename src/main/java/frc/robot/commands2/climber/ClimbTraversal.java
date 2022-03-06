package frc.robot.commands2.climber;

public class ClimbTraversal extends SequentialCommandGroup {
	public ClimbTraversal(ClimberControl climber) {
		addCommands(
			new ClimbMid(control),
			new ClimbStep(control),
			new ClimbStep(control)
		);
	}
}

