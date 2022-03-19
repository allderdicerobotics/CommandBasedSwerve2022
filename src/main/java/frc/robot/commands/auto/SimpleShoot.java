package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
// import frc.robot.commands.intake.ShootWithWait;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.ActuateWOneMotor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.helpers.TimedDrive;
import frc.robot.subsystems.Indexer;
// import frc.robot.subsystems.IntakeActuation;
import frc.robot.subsystems.IntakeShooter;

public class SimpleShoot extends SequentialCommandGroup {
        public SimpleShoot(DriveSubsystem drive) {// , Indexer indexer, IntakeShooter shooter, ActuateWOneMotor
                                                  // leftActuator) {
                addRequirements(drive);// , indexer, shooter, leftActuator);

                addCommands(
                                // new InstantCommand(drive::stop),
                                // new InstantCommand(drive::resetEncoders),
                                // new DriveTimeCommand(
                                // drive,
                                // AutoConstants.AUTO_X_SPEED,
                                // AutoConstants.AUTO_Y_SPEED,
                                // AutoConstants.AUTO_ROTATION_SPEED,
                                // AutoConstants.DEPOSIT_DRIVE_TIME),
                                // new Shoot(shooter, indexer),
                                new WaitCommand(7),
                                new TimedDrive(drive,
                                                AutoConstants.AUTO_X_SPEED, AutoConstants.AUTO_Y_SPEED,
                                                AutoConstants.AUTO_ROTATION_SPEED, false,
                                                (int) AutoConstants.DEPOSIT_DRIVE_TIME
                                                                + (int) AutoConstants.LEAVE_TARMAC_DRIVE_TIME));
                // new DriveTimeCommand(
                // drive,
                // AutoConstants.AUTO_X_SPEED,
                // AutoConstants.AUTO_Y_SPEED,
                // AutoConstants.AUTO_ROTATION_SPEED,
                // AutoConstants.DEPOSIT_DRIVE_TIME
                // + AutoConstants.LEAVE_TARMAC_DRIVE_TIME));
        }
}
