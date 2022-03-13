package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class FrontCamera extends SubsystemBase {

  public PhotonCamera camera = new PhotonCamera("photonvision");
}
