// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.intake.DownAndIn;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeActuation;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final IntakeActuation intakeActuation = new IntakeActuation();
  private final IntakeShooter intakeShooter = new IntakeShooter();
  private final Indexer indexer = new Indexer();

  private final RigidClimbers rigidClimbers = new RigidClimbers();
  private final RotatingClimbers rotatingClimbers = new RotatingClimbers();

  // The driver's controller
  PS4Controller driverController = new PS4Controller(
    OIConstants.kDriverControllerPort
  );
  private final Joystick buttonBoard = new Joystick(
    OIConstants.kOperatorControllerPort
  );
  private Trajectory trajectory = new Trajectory();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    robotInit();
    configureButtonBindings();
    // getAutonomousCommand();

    // Configure default commands
    robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () ->
          robotDrive.driveWithJoystick(
            driverController.getLeftX(),
            driverController.getLeftY(),
            driverController.getRightX(),
            false
          ),
        robotDrive
      )
    );
  }

  // get trajectory json
  private void robotInit() {
    try {
      Path trajectoryPath = Filesystem
        .getDeployDirectory()
        .toPath()
        .resolve("src/main/deploy/Simplehy.wpilib.json");
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      System.out.println("no trajectory");
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of
   * its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */

  // assign intake button
  private void configureButtonBindings() {
    new JoystickButton(driverController, 1)
    .whileActiveOnce(new DownAndIn(intakeActuation, intakeShooter, indexer));

    // assign lift intake button
    new JoystickButton(driverController, 2)
    .whenHeld(
        new InstantCommand(intakeActuation::setPositionUp, intakeActuation)
      );

    // new JoystickButton(driverController, 3).whenHeld(
    // new StartEndCommand(() -> {
    // rigidClimbers.setSpeed(0.25);
    // }, () -> {
    // rigidClimbers.setSpeed(0);
    // }, rigidClimbers));

    // new JoystickButton(driverController, 4).whenHeld(
    // new StartEndCommand(() -> {
    // rigidClimbers.setSpeed(-0.25);
    // }, () -> {
    // rigidClimbers.setSpeed(0);
    // }, rigidClimbers));

    // assign button to rotate arm forward when held
    new JoystickButton(driverController, 5)
    .whenHeld(
        new StartEndCommand(
          () -> {
            rotatingClimbers.setSpeed(0.25);
          },
          () -> {
            rotatingClimbers.setSpeed(0);
          },
          rotatingClimbers
        )
      );

    // assign button to rotate arm backward when held
    new JoystickButton(driverController, 6)
    .whenHeld(
        new StartEndCommand(
          () -> {
            rotatingClimbers.setSpeed(-0.25);
          },
          () -> {
            rotatingClimbers.setSpeed(0);
          },
          rotatingClimbers
        )
      );
    // new JoystickButton(m_driverController, buttonNumber)
    // new JoystickButton(buttonBoard, 2).whileActiveOnce(
    // new InstantCommand(m_intakerRod.SpinIntakerIn())
    // );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
    PathPlannerTrajectory path1 = PathPlanner.loadPath("Example Path", 8, 5);

    var thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController,
      0,
      0,
      AutoConstants.kThetaControllerConstraints
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    var xController = new PIDController(AutoConstants.kPXController, 0, 0);
    var yController = new PIDController(AutoConstants.kPYController, 0, 0);

    // Create a PPSwerveControllerCommand. This is almost identical to WPILib's SwerveControllerCommand, but it uses the holonomic rotation
    // from the PathPlannerTrajectory to control the robot's rotation.
    // See the WPILib SwerveControllerCommand for more info on what you need to pass to the command
    PPSwerveControllerCommand command1 = new PPSwerveControllerCommand(
      path1,
      robotDrive::getPose,
      DriveConstants.kinematics,
      xController,
      yController,
      thetaController,
      robotDrive::setModuleStates,
      robotDrive
    );

    // Reset odometry to the starting pose of the trajectory.
    robotDrive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(
      command1,
      new InstantCommand(() -> robotDrive.drive(0, 0, 0, false), robotDrive)
    );
  }
}
