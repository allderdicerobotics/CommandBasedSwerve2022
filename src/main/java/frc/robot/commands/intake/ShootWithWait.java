package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeShooter;

public class ShootWithWait extends ParallelCommandGroup {
        public ShootWithWait(
                        IntakeShooter intakeShooter,
                        Indexer indexer) {
                addCommands(
                                new StartEndCommand(
                                                indexer::indexerOut,
                                                indexer::stop,
                                                indexer),
                                new WaitCommand(2),
                                new StartEndCommand(
                                                intakeShooter::shootOut,
                                                intakeShooter::stop,
                                                intakeShooter));
        }
}
