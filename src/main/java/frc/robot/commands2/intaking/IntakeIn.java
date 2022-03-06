package frc.robot.commands2.intaking;

public class IntakeIn extends SequentialCommandGroup {
	public IntakeIn(
		IntakeActuation actuation,
		IntakeShooter intakeShooter,
		Indexer indexer
	) {
		addCommands(
			new InstantCommand(actuation::setPositionUp, actuation),
			new RunIntake(intakeShooter, indexer, actuation)
		);
	}
}

