// package frc.robot.commands2.helpers;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.StartEndCommand;
// import frc.robot.subsystems.DriveSubsystem;

// public class DriveHelpers {
// public static CommandBase timedForwards(DriveSubsystem drive, int seconds) {
// return (new StartEndCommand(drive::forwards, drive::stop,
// drive)).withTimeout(seconds);
// }

// public static CommandBase timedBackwards(DriveSubsystem drive, int seconds) {
// return (new StartEndCommand(drive::backwards, drive::stop,
// drive)).withTimeout(seconds);
// }
// }
