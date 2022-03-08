package frc.robot.commands.intaking;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeShooter;

public class RollersIn extends ParallelCommandGroup {
	public RollersIn(
			IntakeShooter intakeShooter,
			Indexer indexer) {
		addCommands(
				new StartEndCommand(
						intakeShooter::runIn,
						intakeShooter::stop,
						intakeShooter),
				new StartEndCommand(
						indexer::indexerIn,
						indexer::stop,
						indexer));
	}
}
