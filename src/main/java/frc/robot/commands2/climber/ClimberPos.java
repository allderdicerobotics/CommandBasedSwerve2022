package frc.robot.commands2.climber;

import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;

import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberPos {
	private final double rigidHeight;
	private final double rotatingAngle;

	public ClimberPos(double rigidHeight, double rotatingAngle) {
		this.rigidHeight = rigidHeight;
		this.rotatingAngle = rotatingAngle;
	}

	public double getRigidHeight() {
		return this.rigidHeight;
	}

	public double getRotatingAngle() {
		return this.rotatingAngle;
	}
}
