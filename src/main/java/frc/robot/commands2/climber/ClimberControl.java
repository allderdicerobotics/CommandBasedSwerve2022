package frc.robot.commands2.climber;

import frc.robot.subsystems.RigidArms;
import frc.robot.subsystems.RotatingArms;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Function;

public class ClimberControl {
	private RigidArms rigidArms;
	private RotatingArms rotatingArms;

	public ClimberControl(RigidArms rigidArms, RotatingArms rotatingArms) {
		this.rigidArms = rigidArms;
		this.rotatingArms = rotatingArms;
	}

	public CommandBase to(ClimberPos position) {
		return (new InstantCommand(
			() -> {
				this.rigidArms.setHeight(position.getRigidHeight());
				this.rotatingArms.setPosition(position.getRotatingAngle());
	
			}
		)).until(() -> (
			this.rigidArms.closeToHeight(position.getRigidHeight())
			&&
			this.rotatingArms.closeToAngle(position.getRotatingAngle())
		)); // if PID is garbage, we'll need some transformation
		    // to check this once and then later to make sure its still all okay
	}

	public RigidArms getRigidArms()		{ return this.rigidArms; }
	public RotatingArms getRotatingArms() 	{ return this.rotatingArms; }
}

