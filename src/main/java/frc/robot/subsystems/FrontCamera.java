package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FrontCamera extends SubsystemBase {
    public PhotonCamera camera = new PhotonCamera("photonvision");
}