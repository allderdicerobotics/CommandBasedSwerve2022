package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FrontCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class OnlyAim extends CommandBase {

  private final DriveSubsystem driveSubsystem;
  private final FrontCamera frontCamera;

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  public OnlyAim(DriveSubsystem driveSubsystem, FrontCamera frontCamera) {
    this.driveSubsystem = driveSubsystem;
    this.frontCamera = frontCamera;
    addRequirements(driveSubsystem, frontCamera);
  }

  @Override
  public void execute() {
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    PhotonPipelineResult result = frontCamera.camera.getLatestResult();
    double rotationSpeed = 0;

    if (result.hasTargets()) {
      // Calculate angular power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed =
        -turnController.calculate(result.getBestTarget().getYaw(), 0);
    }

    driveSubsystem.drive(0, 0, rotationSpeed, false);
  }

  @Override
  public boolean isFinished() {
    return turnController.atSetpoint();
  }
}
