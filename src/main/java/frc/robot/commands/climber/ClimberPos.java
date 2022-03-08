package frc.robot.commands.climber;

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
