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
		this.rigidArms.setHeight(position.getRigidHeight());
		this.rotatingArms.setRotation2d(position.getRotatingAngle());
	}

	public RigidArms getRigidArms()		{ return this.rigidArms; }
	public RotatingArms getRotatingArms() 	{ return this.rotatingArms; }
}

