package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.RigidClimbers;

public class HomeRigidClimbers extends CommandBase {

    private RigidClimbers rigidClimbers;
    private boolean leftHomed;
    private boolean rightHomed;

    public HomeRigidClimbers(RigidClimbers rigidClimbers) {
        this.rigidClimbers = rigidClimbers;
        addRequirements(rigidClimbers);
    }

    @Override
    public void initialize() {
        leftHomed = false;
        rightHomed = false;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Left current", rigidClimbers.getLeftCurrent());
        System.out.println(rigidClimbers.getLeftCurrent());
        SmartDashboard.putNumber("Right current", rigidClimbers.getRightCurrent());
        if ((rigidClimbers.getLeftCurrent() < ClimberConstants.rigidHomeCurrent) && !leftHomed) {
            rigidClimbers.setSpeedLeft(ClimberConstants.rigidHomeSpeed);
            System.out.println("left not homed");
        } else {
            System.out.println("left homed");
            rigidClimbers.setSpeedLeft(0);
            leftHomed = true;
            rigidClimbers.zeroLeft();
        }
        if ((rigidClimbers.getRightCurrent() < ClimberConstants.rigidHomeCurrent) && !rightHomed) {
            rigidClimbers.setSpeedRight(ClimberConstants.rigidHomeSpeed);
        } else {
            System.out.println("right homed");
            rigidClimbers.setSpeedRight(0);
            rightHomed = true;
            rigidClimbers.zeroRight();
        }
    }

    @Override
    public void end(boolean cancelled) {
        rigidClimbers.setSpeedLeft(0);
        rigidClimbers.setSpeedRight(0);
    }

    @Override
    public boolean isFinished() {
        if (leftHomed && rightHomed) {
            System.out.println("zeroed");
        }
        return leftHomed && rightHomed;
    }
}
