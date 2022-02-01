package frc.robot.auto.actions;

import frc.robot.subsystems.*;

/**
 * Uses Vision System to steep to a ball
 * 
 */
public class AutoSteerAction implements Action {
  private Arm mArm  = Arm.getInstance();
  private Drive mDrive = Drive.getInstance();


  public AutoSteerAction(){
  }

  @Override
  public void start() {
  }

  @Override
  public void update() {
    mArm.update();
  }

  @Override
  public boolean isFinished() {
      return false;
  }

  @Override
  public void done() {}
}
