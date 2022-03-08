// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands2.intaking.IntakeCmd;
import frc.robot.commands2.intaking.IntakeDown;
import frc.robot.commands2.intaking.IntakeUp;
import frc.robot.commands2.intaking.RunIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeActuation;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;

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
  private final RigidClimbers rigidClimbers = new RigidClimbers();
  private final RotatingClimbers rotatingClimbers = new RotatingClimbers();
  private final Indexer indexer = new Indexer();
  private final IntakeShooter intakeShooter = new IntakeShooter();
  // private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();

  // The driver's controller
  PS4Controller driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  private final Joystick buttonBoard = new Joystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(() -> robotDrive.driveWithJoystick(driverController.getLeftX(), driverController.getLeftY(),
            driverController.getRightX(), false), robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of
   * its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(driverController, 1).whileActiveOnce(
    // new StartEndCommand(() -> {
    // intakeActuation.setPositionDown();
    // intakeActuation.enable();
    // }, () -> {
    // intakeActuation.setPositionUp();
    // intakeActuation.enable();
    // }, intakeActuation));
    new JoystickButton(driverController, 1).whileActiveOnce(new IntakeCmd(intakeActuation, intakeShooter, indexer));
    // new JoystickButton(driverController, 1).whenHeld(new StartEndCommand(
    // intakeShooter::runIn,
    // intakeShooter::stop,
    // intakeShooter));

    new JoystickButton(driverController, 2).whenHeld(new IntakeUp(intakeActuation));

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

    new JoystickButton(driverController, 5).whenHeld(
        new StartEndCommand(() -> {
          rotatingClimbers.setSpeed(0.25);
        }, () -> {
          rotatingClimbers.setSpeed(0);
        }, rotatingClimbers));

    new JoystickButton(driverController, 6).whenHeld(
        new StartEndCommand(() -> {
          rotatingClimbers.setSpeed(-0.25);
        }, () -> {
          rotatingClimbers.setSpeed(0);
        }, rotatingClimbers));

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
  // public Command getAutonomousCommand() {
  // // Create config for trajectory
  // TrajectoryConfig config =
  // new TrajectoryConfig(
  // AutoConstants.kMaxSpeedMetersPerSecond,
  // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  // // Add kinematics to ensure max speed is actually obeyed
  // .setKinematics(DriveConstants.kDriveKinematics);

  // // An example trajectory to follow. All units in meters.
  // Trajectory exampleTrajectory =
  // TrajectoryGenerator.generateTrajectory(
  // // Start at the origin facing the +X direction
  // new Pose2d(0, 0, new Rotation2d(0)),
  // // Pass through these two interior waypoints, making an 's' curve path
  // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  // // End 3 meters straight ahead of where we started, facing forward
  // new Pose2d(3, 0, new Rotation2d(0)),
  // config);

  // var thetaController =
  // new ProfiledPIDController(
  // AutoConstants.kPThetaController, 0, 0,
  // AutoConstants.kThetaControllerConstraints);
  // thetaController.enableContinuousInput(-Math.PI, Math.PI);

  // SwerveControllerCommand swerveControllerCommand =
  // // to do fix
  // new SwerveControllerCommand(
  // exampleTrajectory,
  // m_robotDrive::getPose, // Functional interface to feed supplier
  // DriveConstants.kDriveKinematics,

  // // Position controllers
  // new PIDController(AutoConstants.kPXController, 0, 0),
  // new PIDController(AutoConstants.kPYController, 0, 0),
  // thetaController,
  // m_robotDrive::setModuleStates,
  // m_robotDrive);

  // // Reset odometry to the starting pose of the trajectory.
  // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  // // Run path following command, then stop at the end.
  // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
  // false));
  // }
}
