package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeActuation;
import frc.robot.subsystems.IntakeShooter;

public class DownAndIn extends ParallelCommandGroup {
	public DownAndIn(
			IntakeActuation actuation,
			IntakeShooter intakeShooter,
			Indexer indexer) {
		addCommands(
				new StartEndCommand(actuation::setPositionDown, actuation::setPositionUp, actuation),
				new RollersIn(intakeShooter, indexer));
	}
}
