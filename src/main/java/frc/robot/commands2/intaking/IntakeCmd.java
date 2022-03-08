package frc.robot.commands2.intaking;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeActuation;
import frc.robot.subsystems.IntakeShooter;

public class IntakeCmd extends ParallelCommandGroup {
	public IntakeCmd(
			IntakeActuation actuation,
			IntakeShooter intakeShooter,
			Indexer indexer) {
		// addRequirements(actuation, intakeShooter, indexer);
		addCommands(
				new StartEndCommand(actuation::setPositionDown, actuation::setPositionUp, actuation),
				new RunIntake(intakeShooter, indexer));
	}
}
