package frc.robot.commands2.intaking;

import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeActuation;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class RunIntake extends ParallelCommandGroup {
	public RunIntake(
			IntakeShooter intakeShooter,
			Indexer indexer) {
		// addRequirements(intakeActuation); // phantom
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
