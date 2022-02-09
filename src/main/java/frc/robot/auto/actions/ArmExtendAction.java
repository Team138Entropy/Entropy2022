package frc.robot.auto.actions;

import frc.robot.subsystems.Arm;

/**
 * Moves the Arm to a desired degrees
 */
public class ArmExtendAction implements Action {
  private Arm mArm  = Arm.getInstance();
  private final boolean mIsExtended;

  public ArmExtendAction(boolean isExtended) {
    mIsExtended = isExtended;
  }

  @Override
  public void start() {}

  @Override
  public void update() {
    if (mIsExtended) mArm.extend();
    else mArm.retract();
  }

  @Override
  public boolean isFinished() {
    if (mIsExtended) return mArm.isExtended();
    else return mArm.isRetracted();
  }

  @Override
  public void done() {}
}
