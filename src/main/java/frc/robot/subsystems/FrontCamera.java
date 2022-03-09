package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public PhotonCamera frontCamera = new PhotonCamera("photonvision");
}
