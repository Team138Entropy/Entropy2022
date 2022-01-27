package frc.robot.auto.actions;

/**
 * Moves the Arm to a desired degrees
 */
public class ArmAction {
    private final double mTargetAngle;

    public ArmAction(double targetAngle){
        mTargetAngle = targetAngle;
    }
}
