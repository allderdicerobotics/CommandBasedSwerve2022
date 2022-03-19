package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeShooter;

public class Shoot extends ParallelRaceGroup {
	public Shoot(
			IntakeShooter intakeShooter,
			Indexer indexer) {
		addCommands(
				new StartEndCommand(
						intakeShooter::shootOut,
						intakeShooter::stop,
						intakeShooter),
				new WaitCommand(0.5),
				new StartEndCommand(
						indexer::indexerOut,
						indexer::stop,
						indexer));
	}
}
