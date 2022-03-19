package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTimeCommand extends SequentialCommandGroup {
        public DriveTimeCommand(DriveSubsystem drive, double xSpeed, double ySpeed, double rotationSpeed,
                        double duration) {
                addRequirements(drive);
                addCommands(
                                new StartEndCommand(
                                                () -> drive.drive(xSpeed, ySpeed,
                                                                rotationSpeed, false),
                                                () -> drive.stop(),
                                                drive).withTimeout(duration));
        }
}
