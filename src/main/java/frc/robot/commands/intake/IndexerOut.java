package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeShooter;

public class IndexerOut extends ParallelCommandGroup {
    public IndexerOut(
            Indexer indexer) {
        addCommands(
                new StartEndCommand(
                        indexer::indexerOut,
                        indexer::stop,
                        indexer));
    }
}
