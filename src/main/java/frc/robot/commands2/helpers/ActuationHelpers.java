package frc.robot.commands2.helpers;

import frc.robot.subsystems.IntakeActuation;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;

public static class ActuationHelpers {
	public static ParallelRaceGroup checkedTo(IntakeActuation actuation, ActuationT to) {
		return uncheckedTo(actuation).until(
				() -> {
					switch (to) {
						case ActuationT.UP:
							return actuation.isUp();
							break;
						case ActuationT.DOWN:
							return actuation.isDown();
							break;
						default:
							throw new Exception("unreachable");
					}
				});
	}

	public static CommandBase uncheckedTo(IntakeActuation actuation, ActuationT to) {
		switch (to) {
			case ActuationT.UP:
				return new InstantCommand(actuation::setPositionUp);
				break;
			case ActuationT.DOWN:
				return new InstantCommand(actuation::setPositionDown);
				break;
			default:
				throw new Exception("unreachable");
		}
	}
}
