package frc.robot.commands2.climber;

import frc.robot.subsystems.RigidArms;
import frc.robot.subsystems.RotatingArms;

import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberPos {
	private final double rigidHeight;
	private final Rotation2d rotatingAngle;

	public ClimberPos(double rigidHeight, Rotation2d rotatingAngle) {
		this.rigidHeight = rigidHeight;
		this.rotatingAngle = rotatingAngle;
	}

	public double getRigidHeight()		{ return this.rigidHeight; }
	public double getRotatingAngle()	{ return this.rotatingAngle; }
}

