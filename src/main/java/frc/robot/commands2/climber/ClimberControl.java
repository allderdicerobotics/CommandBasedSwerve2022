package frc.robot.commands2.climber;

import frc.robot.subsystems.RigidClimbers;
import frc.robot.subsystems.RotatingClimbers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimberControl {
	private RigidClimbers rigidArms;
	private RotatingClimbers rotatingArms;

	public ClimberControl(RigidClimbers rigidArms, RotatingClimbers rotatingArms) {
		this.rigidArms = rigidArms;
		this.rotatingArms = rotatingArms;
	}

	// public CommandBase to(ClimberPos position) {
	// return (new InstantCommand(
	// () -> {
	// this.rigidArms.setHeight(position.getRigidHeight());
	// this.rotatingArms.closeToAngle(position.getRotatingAngle());

	// }
	// )).until(() -> (
	// this.rigidArms.closeToHeight(position.getRigidHeight())
	// &&
	// this.rotatingArms.closeToAngle(position.getRotatingAngle())
	// )); // if PID is garbage, we'll need some transformation
	// // to check this once and then later to make sure its still all okay
	// }

	public RigidClimbers getRigidArms() {
		return this.rigidArms;
	}

	public RotatingClimbers getRotatingArms() {
		return this.rotatingArms;
	}
}
