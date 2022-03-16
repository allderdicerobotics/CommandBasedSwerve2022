// package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.ClimberConstants;
// import frc.robot.subsystems.RigidClimbers;

// public class RigidLimitSwitchZero extends CommandBase {

// DigitalInput leftLimitSwitch = new
// DigitalInput(ClimberConstants.leftRigidLimitSwitchPort);
// DigitalInput rightLimitSwitch = new
// DigitalInput(ClimberConstants.rightRigidLimitSwitchPort);
// private RigidClimbers rigidClimbers;
// private boolean leftZeroed;
// private boolean rightZeroed;

// public RigidLimitSwitchZero(RigidClimbers rigidClimbers) {
// this.rigidClimbers = rigidClimbers;
// addRequirements(rigidClimbers);
// }

// @Override
// public void initialize() {
// leftZeroed = false;
// rightZeroed = false;
// }

// @Override
// public void execute() {
// if (!leftZeroed) {
// rigidClimbers.setSpeedLeft(ClimberConstants.rigidHomeSpeed);
// }
// if (rigidClimbers.getLeftSpeed() > 0) {
// if (leftLimitSwitch.get()) {
// rigidClimbers.zeroLeft();
// rigidClimbers.setSpeedLeft(0);
// leftZeroed = true;
// }
// }
// if (!rightZeroed) {
// rigidClimbers.setSpeedRight(ClimberConstants.rigidHomeSpeed);
// }
// if (rigidClimbers.getRightSpeed() > 0) {
// if (rightLimitSwitch.get()) {
// rigidClimbers.zeroRight();
// rigidClimbers.setSpeedRight(0);
// rightZeroed = true;
// }
// }
// }
// }
