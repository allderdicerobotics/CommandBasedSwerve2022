package frc.robot.commands2.intaking;

public class IntakeIn extends SequentialCommandGroup {
	public IntakeIn(
		IntakeActuation actuation,
		IntakeShooter intakeShooter,
		Indexer indexer
	) {
		addRequirements(actuation, intakeShooter, indexer);
		addCommands(
			new InstantCommand(actuation::setPosositionUp, actuation),
			new RunIntake(intakeShooter, indexer, actuation)
		);
	}
}

