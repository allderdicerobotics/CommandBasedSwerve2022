// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.sql.Driver;
// import java.util.List;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auto.SimpleShoot;
import frc.robot.commands.climber.ClimbHigh;
import frc.robot.commands.climber.ClimbMid;
import frc.robot.commands.climber.ClimbStep;
import frc.robot.commands.climber.ClimbTraversal;
import frc.robot.commands.climber.ClimbersToPosition;
import frc.robot.commands.climber.HomeRigidClimbers;
import frc.robot.commands.climber.StopClimbers;
import frc.robot.commands.climber.steppt2;
import frc.robot.commands.intake.DownAndIn;
import frc.robot.commands.intake.RollersIn;
import frc.robot.commands.intake.RollersOut;
import frc.robot.commands.shooter.BackwardsShoot;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.ActuateWOneMotor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeActuation;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem robotDrive = new DriveSubsystem();

  // public IntakeActuation intakeActuation; // = new~ IntakeActuation();
  // public final IntakeShooter intakeShooter = new IntakeShooter();
  // public final Indexer indexer = new Indexer();
  // public final ActuateWOneMotor leftActuation = new ActuateWOneMotor();

  private final RigidClimbers rigidClimbers = new RigidClimbers();
  private final RotatingClimbers rotatingClimbers = new RotatingClimbers();

  // The driver's controller
  PS4Controller driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  private final Joystick buttonBoard = new Joystick(OIConstants.kOperatorControllerPort);
  private Trajectory trajectory = null;

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
        new RunCommand(() -> robotDrive.driveWithJoystick(
            driverController.getLeftX(),
            driverController.getLeftY(),
            driverController.getRightX(), (() -> this.getFocState())), robotDrive));
  }

  private boolean getFocState() {
    return this.robotDrive.evalFocOverride(true);
  }

  private void toggleFocState() {
    this.robotDrive.toggleFocOverride();
  }

  // get trajectory json
  private void robotInit() {
    // leftActuation.setBrake();

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("src/main/deploy/Simplehy.wpilib.json");
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      System.out.println("no trajectory");
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * calling passing it
   * to a {@link JoystickButton}.
   */

  // assign intake button
  private void configureButtonBindings() {
    new JoystickButton(driverController, 5).whenPressed(new InstantCommand(() -> {
      this.robotDrive.toggleFocOverride();
      System.out.println("TOGGLED");
    }));

    // new JoystickButton(driverController, 4).whenHeld(
    // new StartEndCommand(() -> {
    // if (intakeActuation == null) {
    // intakeActuation = new IntakeActuation();
    // }
    // intakeActuation.setSpeed(-0.25);
    // }, () -> {
    // rigidClimbers.setSpeed(0);
    // }, rigidClimbers));

    // new JoystickButton(driverController, 2).whenHeld(
    // new StartEndCommand(() -> {
    // if (intakeActuation == null) {
    // intakeActuation = new IntakeActuation();
    // }
    // intakeActuation.setSpeed(0.25);
    // }, () -> {
    // rigidClimbers.setSpeed(0);
    // }, rigidClimbers));

    // new JoystickButton(driverController, 1).whileActiveOnce(new
    // RollersIn(intakeShooter, indexer));

    // new JoystickButton(driverController, 3).whileActiveOnce(new
    // Shoot(intakeShooter, indexer));

    // new JoystickButton(driverController, 5).whileActiveOnce(new
    // PrintCommand("halfway shoot")); // TODO

    // new JoystickButton(driverController, 5).whenPressed(new InstantCommand(() ->
    // this.toggleFocState()));

    // new JoystickButton(driverController, 6).whileActiveOnce(new InstantCommand(()
    // -> {
    // if (intakeActuation == null) {
    // intakeActuation = new IntakeActuation();
    // }
    // (new DownAndIn(intakeActuation, intakeShooter, indexer)).schedule();
    // }));

    /*
     * new JoystickButton(buttonBoard, 4).whenHeld(
     * new StartEndCommand(() -> {
     * rigidClimbers.setSpeed(0.25);
     * }, () -> {
     * rigidClimbers.setSpeed(0);
     * }, rigidClimbers));
     * 
     * new JoystickButton(buttonBoard, 7).whenHeld(
     * new StartEndCommand(() -> {
     * rigidClimbers.setSpeed(-0.25);
     * }, () -> {
     * rigidClimbers.setSpeed(0);
     * }, rigidClimbers));
     * 
     * new JoystickButton(buttonBoard, 3).whenHeld(
     * new StartEndCommand(() -> {
     * rotatingClimbers.setSpeed(0.25);
     * }, () -> {
     * rotatingClimbers.setSpeed(0);
     * }, rigidClimbers));
     * 
     * new JoystickButton(buttonBoard, 1).whenHeld(
     * new StartEndCommand(() -> {
     * rotatingClimbers.setSpeed(-0.25);
     * }, () -> {
     * rotatingClimbers.setSpeed(0);
     * }, rigidClimbers));
     */

    // new JoystickButton(buttonBoard, 11).whenPressed(new InstantCommand(() -> {
    // if (intakeActuation == null) {
    // intakeActuation = new IntakeActuation();
    // }
    // intakeActuation.setPositionUp();
    // }));

    // new JoystickButton(buttonBoard, 12).whenPressed(new InstantCommand(() -> {
    // if (intakeActuation == null) {
    // intakeActuation = new IntakeActuation();
    // }
    // intakeActuation.setPositionDown();
    // leftActuation.setSpeed(1.0);
    // }));

    // new JoystickButton(buttonBoard, 12).whenHeld(
    // new StartEndCommand(() -> {
    // leftActuation.setSpeed(1.00);
    // }, () -> {
    // leftActuation.setSpeed(0);
    // }, rigidClimbers));

    new JoystickButton(buttonBoard, 6).whenPressed(new ClimbMid(rigidClimbers, rotatingClimbers));

    new JoystickButton(buttonBoard, 5).whenPressed(new ClimbStep(rigidClimbers, rotatingClimbers));

    new JoystickButton(buttonBoard, 8).whenPressed(new ClimbTraversal(rigidClimbers, rotatingClimbers));

    new JoystickButton(buttonBoard, 4).whenHeld(
        new StartEndCommand(() -> {
          rigidClimbers.setSpeed(0.9);
        }, () -> {
          rigidClimbers.setSpeed(0);
          SmartDashboard.putNumber("climber rigid value", rigidClimbers.getEncoderValue());
        }, rigidClimbers));

    new JoystickButton(buttonBoard, 7).whenHeld(
        new StartEndCommand(() -> {
          rigidClimbers.setSpeed(-0.9);
        }, () -> {
          rigidClimbers.setSpeed(0);
          SmartDashboard.putNumber("climber rigid value", rigidClimbers.getEncoderValue());
        }, rigidClimbers));

    new JoystickButton(buttonBoard, 3).whenHeld(
        new StartEndCommand(() -> {
          rotatingClimbers.setSpeed(0.25);
        }, () -> {
          rotatingClimbers.setSpeed(0);
        }, rigidClimbers));

    new JoystickButton(buttonBoard, 1).whenHeld(
        new StartEndCommand(() -> {
          rotatingClimbers.setSpeed(-0.25);
        }, () -> {
          rotatingClimbers.setSpeed(0);
        }, rigidClimbers));

    // new JoystickButton(buttonBoard, 11).whenPressed(new
    // InstantCommand(intakeActuation::setPositionUp));

    // new JoystickButton(buttonBoard, 12).whenPressed(new
    // InstantCommand(intakeActuation::setPositionDown));

    new JoystickButton(buttonBoard, 2).whenPressed(new StopClimbers(rigidClimbers, rotatingClimbers));

    // private void configureButtonBindings() {
    // new JoystickButton(driverController, 1).whileActiveOnce(
    // new RollersIn(intakeShooter, indexer));

    // new JoystickButton(driverController, 2).whileActiveOnce(
    // new ShootOut(intakeShooter, indexer));

    // new JoystickButton(driverController, 3).whileActiveOnce(
    // new InstantCommand(intakeActuation::setPositionUp));

    // new JoystickButton(driverController, 4).whileActiveOnce(
    // new InstantCommand(intakeActuation::setPositionDown));

    // UNCOMMENT
    // new JoystickButton(driverController, 1).whenPressed(new
    // ClimbMid(rigidClimbers, rotatingClimbers));

    // new JoystickButton(driverController, 2).whenPressed(new
    // ClimbStep(rigidClimbers, rotatingClimbers));

    // new JoystickButton(driverController, 1).whileActiveOnce(
    // new DownAndIn(intakeActuation, intakeShooter, indexer));
    // new JoystickButton(driverController, 1)
    // .whenPressed(new ClimbMid(rigidClimbers, rotatingClimbers));
    // new JoystickButton(driverController, 1)
    // .whenPressed(new InstantCommand(intakeActuation::setPositionDown,
    // intakeActuation));

    // new JoystickButton(driverController, 3)
    // .whenPressed(new steppt2(rigidClimbers, rotatingClimbers));

    // assign lift intake button
    // new JoystickButton(driverController, 2)
    // .whenPressed(new ClimbStep(rigidClimbers, rotatingClimbers));

    // new JoystickButton(driverController, 2)
    // .whenPressed(new ClimbersToPosition(
    // rigidClimbers,
    // rotatingClimbers,
    // ClimberConstants.RIGID_CLIMBERS_RELEASE_BAR,
    // ClimberConstants.ROTATING_CLIMBERS_ON_BAR));

    // new JoystickButton(driverController, 2).whenHeld(
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
    // new JoystickButton(driverController, 5).whenHeld(
    // new StartEndCommand(() -> {
    // rotatingClimbers.setSpeed(0.25);
    // }, () -> {
    // rotatingClimbers.setSpeed(0);
    // }, rotatingClimbers));

    // // assign button to rotate arm backward when held
    // new JoystickButton(driverController, 6).whenHeld(
    // new StartEndCommand(() -> {
    // rotatingClimbers.setSpeed(-0.25);
    // }, () -> {
    // rotatingClimbers.setSpeed(0);
    // }, rotatingClimbers));

    // new JoystickButton(driverController, 8).whenHeld(
    // new ClimbersToPosition(
    // rigidClimbers,
    // rotatingClimbers,
    // ClimberConstants.RIGID_CLIMBERS_MIN,
    // ClimberConstants.ROTATING_CLIMBERS_F_SM));

    // new JoystickButton(driverController, 1).whenHeld(
    // new StartEndCommand(() -> {
    // rigidClimbers.setSpeedLeft(-0.25);
    // }, () -> {
    // rigidClimbers.setSpeedLeft(0);
    // }, rigidClimbers));

    // new JoystickButton(driverController, 2).whenHeld(
    // new StartEndCommand(() -> {
    // rigidClimbers.setSpeedLeft(0.25);
    // }, () -> {
    // rigidClimbers.setSpeedLeft(0);
    // }, rigidClimbers));

    // new JoystickButton(driverController, buttonNumber);
    // new JoystickButton(buttonBoard, 2).whileActiveOnce(
    // new InstantCommand(intakerRod.SpinIntakerIn())
    // );

    // new JoystickButton(buttonBoard, 3)
    // .whenHeld(new DownAndIn(intakeActuation, intakeShooter, indexer)); // not acc
    // port 3
    // new JoystickButton(buttonBoard, 4).whenHeld(new Shoot(intakeShooter,
    // indexer));
    // new JoystickButton(buttonBoard, 5)
    // .whenHeld(new BackwardsShoot(intakeActuation, intakeShooter, indexer));
    // new JoystickButton(buttonBoard, 6).whenPressed(new
    // RunCommand(intakeActuation::setPositionUp));
    // new JoystickButton(buttonBoard, 7)
    // .whenPressed(new RunCommand(intakeActuation::setPositionDown));
    // new JoystickButton(buttonBoard, 8).whenPressed(new
    // RunCommand(intakeActuation::setPositionUp));
    // new JoystickButton(buttonBoard, 9).whenPressed(new ClimbMid(rigidClimbers,
    // rotatingClimbers));
    // new JoystickButton(buttonBoard, 10).whenPressed(new ClimbStep(rigidClimbers,
    // rotatingClimbers));
    // new JoystickButton(buttonBoard, 11).whenPressed(new ClimbHigh(rigidClimbers,
    // rotatingClimbers));
    // new JoystickButton(buttonBoard, 12).whenPressed(new
    // RunCommand(rigidClimbers::upHalfSpeed));
    // new JoystickButton(buttonBoard, 13).whenPressed(new
    // RunCommand(rigidClimbers::downHalfSpeed));
    // new JoystickButton(buttonBoard, 14).whenPressed(new
    // RunCommand(rotatingClimbers::upHalfSpeed));
    // new JoystickButton(buttonBoard, 15)
    // .whenPressed(new RunCommand(rotatingClimbers::downHalfSpeed));
    // new JoystickButton(buttonBoard, 16)
    // .whenPressed(new StopClimbers(rigidClimbers, rotatingClimbers));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  // follow trajectory with PID
  public Command getAutonomousCommand() {
    return new SimpleShoot(
        this.robotDrive
    // nvm this.robotContainer.intakeActuation,
    // this.indexer,
    // this.intakeShooter,
    // this.leftActuation
    );
  }

  // Create config for trajectory
  // TrajectoryConfig config = new TrajectoryConfig(
  // AutoConstants.kMaxSpeedMetersPerSecond,
  // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  // Add kinematics to ensure max speed is actually obeyed
  // .setKinematics(DriveConstants.kinematics);

  // An example trajectory to follow. All units in meters.
  // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  // Start at the origin facing the +X direction
  // new Pose2d(0, 0, new Rotation2d(0)),
  // Pass through these two interior waypoints, making an 's' curve path
  // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  // End 3 meters straight ahead of where we started, facing forward
  // new Pose2d(3, 0, new Rotation2d(0)),
  // config);

  // var thetaController = new ProfiledPIDController(
  // AutoConstants.kPThetaController, 0, 0,
  // AutoConstants.kThetaControllerConstraints);
  // thetaController.enableContinuousInput(-Math.PI, Math.PI);

  // SwerveControllerCommand swerveControllerCommand = new
  // SwerveControllerCommand(
  // this.trajectory,
  // robotDrive::getPose, // Functional interface to feed supplier
  // DriveConstants.kinematics,

  // // Position controllers
  // new PIDController(AutoConstants.kPXController, 0, 0),
  // new PIDController(AutoConstants.kPYController, 0, 0),
  // thetaController,
  // robotDrive::setModuleStates,
  // robotDrive);

  // // Reset odometry to the starting pose of the trajectory.
  // robotDrive.resetOdometry(trajectory.getInitialPose());

  // // Run path following command, then stop at the end.
  // return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0,
  // false));

}
