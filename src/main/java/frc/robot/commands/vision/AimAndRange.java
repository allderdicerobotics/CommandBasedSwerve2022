package frc.robot.commands.vision;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FrontCamera;

public class AimAndRange extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final FrontCamera frontCamera;

    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    public AimAndRange(DriveSubsystem driveSubsystem, FrontCamera frontCamera) {
        this.driveSubsystem = driveSubsystem;
        this.frontCamera = frontCamera;
        addRequirements(driveSubsystem, frontCamera);
    }

    @Override
    public void execute() {
        // Vision-alignment mode
        // Query the latest result from PhotonVision
        PhotonPipelineResult result = frontCamera.camera.getLatestResult();

        double forwardSpeed = 0;
        double rotationSpeed = 0;

        if (result.hasTargets()) {
            // First calculate range
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                    VisionConstants.CAMERA_HEIGHT_METERS,
                    VisionConstants.TARGET_HEIGHT_METERS,
                    VisionConstants.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));

            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            forwardSpeed = -forwardController.calculate(range, VisionConstants.GOAL_RANGE_METERS);

            // Also calculate angular power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        }

        driveSubsystem.drive(forwardSpeed, 0, rotationSpeed, false);
    }

    @Override
    public boolean isFinished() {
        return forwardController.atSetpoint() && turnController.atSetpoint();
    }

}
