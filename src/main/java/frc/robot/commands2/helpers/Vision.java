package frc.robot.commands2.helpers;

import java.util.Optional;
import java.util.stream.*;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.*;
import org.opencv.core.Point;

import frc.robot.VectorDef;
import frc.robot.PointVec;

public class Vision {
	PhotonCamera cam;

	public Vision(PhotonCamera cam) {
		this.cam = cam;
	}

	public Optional<Point> targetFramePosition(PhotonCamera cam) {
		PhotonPipelineResult res = cam.getLastResult();
		if (!res.hasTargets()) {
			return Optional.empty();
		} // implicit else ...
		Stream<TrackedCorner> trackedCorners = res
							.getBestResult()
							.getCorners()
							.stream();
		Stream<Point> cornerPoints = trackedCorners.map(
			(TrackedCorner corner) -> new Point(corner.x, corner.y)
		);
		VectorDef<Point> vec = new PointVec();
		Point meanPosition = vec.mean(cornerPoints);
		return Optional.of(meanPosition);
	}
}
