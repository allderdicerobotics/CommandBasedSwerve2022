package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTimeCommand extends SequentialCommandGroup {
    public DriveTimeCommand(DriveSubsystem drive, double speed, double duration) {
        addRequirements(drive);
        addCommands(
                new StartEndCommand(
                        () -> drive.drive(0.0, speed, 0.0, false),
                        () -> drive.stop(),
                        drive).withTimeout(duration));
    }
}
