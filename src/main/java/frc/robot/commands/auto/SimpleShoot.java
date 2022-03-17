package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.intake.ShootWithWait;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeActuation;
import frc.robot.subsystems.IntakeShooter;

public class SimpleShoot extends SequentialCommandGroup {
    public SimpleShoot(DriveSubsystem drive, IntakeActuation actuation, Indexer indexer, IntakeShooter shooter) {
        addRequirements(drive, actuation, indexer, shooter);
        addCommands(
                new DriveTimeCommand(
                        drive,
                        -AutoConstants.AUTO_DRIVE_SPEED,
                        AutoConstants.DEPOSIT_DRIVE_TIME),
                new ShootWithWait(shooter, indexer),
                new DriveTimeCommand(
                        drive,
                        AutoConstants.AUTO_DRIVE_SPEED,
                        AutoConstants.DEPOSIT_DRIVE_TIME + AutoConstants.LEAVE_TARMAC_DRIVE_TIME));
    }
}
