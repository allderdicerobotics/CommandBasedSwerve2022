// package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.ClimberConstants;
// import frc.robot.subsystems.RotatingClimbers;

// public class RotatingLimitSwitchZero extends CommandBase {

// DigitalInput leftLimitSwitch = new
// DigitalInput(ClimberConstants.leftRotatingLimitSwitchPort);
// DigitalInput rightLimitSwitch = new
// DigitalInput(ClimberConstants.rightRotatingLimitSwitchPort);
// private RotatingClimbers rotatingClimbers;
// private boolean leftZeroed;
// private boolean rightZeroed;

// public RotatingLimitSwitchZero(RotatingClimbers rotatingClimbers) {
// this.rotatingClimbers = rotatingClimbers;
// addRequirements(rotatingClimbers);
// }

// @Override
// public void initialize() {
// leftZeroed = false;
// rightZeroed = false;
// }

// @Override
// public void execute() {
// if (!leftZeroed) {
// rotatingClimbers.setSpeedLeft(ClimberConstants.rigidHomeSpeed);
// }
// if (rotatingClimbers.getLeftSpeed() > 0) {
// if (leftLimitSwitch.get()) {
// rotatingClimbers.zeroLeft();
// rotatingClimbers.setSpeedLeft(0);
// leftZeroed = true;
// }
// }
// if (!rightZeroed) {
// rotatingClimbers.setSpeedRight(ClimberConstants.rigidHomeSpeed);
// }
// if (rotatingClimbers.getRightSpeed() > 0) {
// if (rightLimitSwitch.get()) {
// rotatingClimbers.zeroRight();
// rotatingClimbers.setSpeedRight(0);
// rightZeroed = true;
// }
// }
// }
// }
