package frc.robot.auto.actions;

import frc.robot.subsystems.Arm;

/**
 * Moves the Arm to a desired degrees
 */
public class ArmExtendAction implements Action {
  private Arm mArm  = Arm.getInstance();
  private final double mTargetExtension;

  public ArmExtendAction(double targetExtension){
    mTargetExtension = targetExtension;
  }

  @Override
  public void start() {
    mArm.extendToPosition(mTargetExtension);
  }

  @Override
  public void update() {
    mArm.update();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(mArm.getExtension() - mTargetExtension) < 2;
  }

  @Override
  public void done() {}
}
