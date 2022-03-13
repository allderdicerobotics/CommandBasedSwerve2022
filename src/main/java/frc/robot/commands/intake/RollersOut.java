package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeShooter;

public class RollersOut extends ParallelCommandGroup {
	public RollersOut(
			IntakeShooter intakeShooter,
			Indexer indexer) {
		addCommands(
				new StartEndCommand(
						intakeShooter::runOutSlow,
						intakeShooter::stop,
						intakeShooter),
				new StartEndCommand(
						indexer::indexerOut,
						indexer::stop,
						indexer));
	}
}