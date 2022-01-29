package frc.robot.auto.actions;

import frc.robot.subsystems.Arm;

/**
 * Moves the Arm to a desired degrees
 */
public class ArmRotateAction implements Action {
  private Arm mArm  = Arm.getInstance();
  private final double mTargetAngle;

  public ArmRotateAction(double targetAngle){
    mTargetAngle = targetAngle;
  }

  @Override
  public void start() {}

  @Override
  public void update() {
    mArm.update(mTargetAngle);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(mArm.getRotation() - mTargetAngle) < 3;
  }

  @Override
  public void done() {}
}
