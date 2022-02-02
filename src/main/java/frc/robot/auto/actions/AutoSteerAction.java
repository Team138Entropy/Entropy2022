package frc.robot.auto.actions;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.vision.*;

/**
 * Uses Vision System to steer to a ball
 * Runs the auto steer loop until ball is in cargo, or timeed out
 * periodically plot positions to then reverse out?
 */
public class AutoSteerAction implements Action {
  private Arm mArm  = Arm.getInstance();
  private Drive mDrive = Drive.getInstance();
  private VisionManager mVisionManager = VisionManager.getInstance();
  private boolean mComplete;


  public AutoSteerAction(){
      mComplete = false;
  }

  @Override
  public void start() {


  }

  @Override
  public void update() {
    TargetInfo ti = mVisionManager.getTarget(Constants.TargetType.CAMERA_1_BLUE_CARGO, Constants.Vision.kAllowedSecondsThreshold);
    if(ti != null){
        // steer to the target
    }else{
        // no vision packet, must be done? or this is an error
        mComplete = true;
    }

    // check if ball is in grasper
      
  }

  @Override
  public boolean isFinished() {
      return mComplete;
  }

  @Override
  public void done() {}
}
