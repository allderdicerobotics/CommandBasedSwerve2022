package frc.robot.helpers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class TimedDrive extends SequentialCommandGroup {
    public TimedDrive(DriveSubsystem drive, double xSpeed, double ySpeed,
            double rot,
            boolean fieldRelative, int seconds) {
        addRequirements(drive);
        addCommands(
                new RunCommand(() -> {
                    drive.drive(xSpeed, ySpeed, rot, fieldRelative);
                }, drive).withTimeout(seconds),
                new InstantCommand(() -> {
                    drive.drive(0, 0, 0, false);
                }));
    }
}
