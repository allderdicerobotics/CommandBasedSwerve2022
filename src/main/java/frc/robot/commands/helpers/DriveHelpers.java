package frc.robot.commands.helpers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveHelpers {
    public static CommandBase timedDrive(DriveSubsystem drive, double xSpeed, double ySpeed, double rot,
            boolean fieldRelative, int seconds) {
        return (new StartEndCommand(
                () -> {
                    drive.drive(xSpeed, ySpeed, rot, fieldRelative);
                }, () -> {
                    drive.drive(0, 0, 0, false);
                },
                drive)).withTimeout(seconds);
    }
}
