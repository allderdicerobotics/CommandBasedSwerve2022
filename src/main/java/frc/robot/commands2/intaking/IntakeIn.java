package frc.robot.commands2.intaking;

import frc.robot.subsystems.IntakeActuation;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands2.helpers.*;

public class IntakeIn extends SequentialCommandGroup {
	public IntakeIn(
		IntakeActuation actuation,
		IntakeShooter intakeShooter,
		Indexer indexer
	) {
		addCommands(
			ActuationHelpers.toUnchecked(actuation, ActuationT.DOWN),
			new RunIntake(intakeShooter, indexer, actuation)
		);
	}
}

