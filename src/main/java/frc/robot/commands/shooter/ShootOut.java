package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeShooter;

public class ShootOut extends ParallelCommandGroup {
	public ShootOut(
			IntakeShooter intakeShooter,
			Indexer indexer) {
		addCommands(
				new StartEndCommand(
						intakeShooter::shootOut,
						intakeShooter::stop,
						intakeShooter),
				new StartEndCommand(
						indexer::indexerInSlow,
						indexer::stop,
						indexer));
	}
}
