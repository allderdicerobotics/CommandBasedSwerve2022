package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeActuation;
import frc.robot.subsystems.IntakeShooter;

public class BackwardsShoot extends SequentialCommandGroup {
	public BackwardsShoot(
			IntakeActuation actuation,
			IntakeShooter intakeShooter,
			Indexer indexer) {
		addCommands(
				new InstantCommand(() -> {
					actuation.setPosition(1.2);
				}, actuation),
				new WaitCommand(1),
				new Shoot(intakeShooter, indexer));
	}
}
