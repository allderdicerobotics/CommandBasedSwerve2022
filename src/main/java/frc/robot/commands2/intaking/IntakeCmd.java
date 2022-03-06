package frc.robot.commands2.intaking;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeActuation;
import frc.robot.subsystems.IntakeShooter;

public class IntakeCmd extends SequentialCommandGroup {
	public IntakeCmd(
			IntakeActuation actuation,
			IntakeShooter intakeShooter,
			Indexer indexer) {
		addRequirements(actuation, intakeShooter, indexer);
		// addCommands(
		// new InstantCommand(actuation::setPosositionUp, actuation),
		// new RunIntake(intakeShooter, indexer, actuation)
		// );
	}
}
