package frc.robot.commands2.intaking;

public class RunIntake extends ParallelCommandGroup {
	public RunIntake(
		IntakeShooter intakeShooter,
		Indexer indexer,
		IntakeActuation intakeActuation
	) {
		addRequirements(intakeActuation); // phantom
	}
}

