package frc.robot.auto.actions;

import frc.robot.subsystems.Arm;

/**
 * Moves the Arm to a desired degrees
 */
public class ArmRotateAction implements Action {
  private final Arm mArm  = Arm.getInstance();
  private final double mTargetAngle;

  public ArmRotateAction(double targetAngle){
    mTargetAngle = targetAngle;
  }

  @Override
  public void start() {}

  @Override
  public void update() {
    mArm.rotateToPosition(mTargetAngle);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(mArm.getRotation() - mTargetAngle) < 10;
  }

  @Override
  public void done() {}
}
