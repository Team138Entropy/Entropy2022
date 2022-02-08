package frc.robot.auto.actions;

import java.lang.annotation.Target;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.vision.*;

/**
 * Uses Vision System to steer to a ball
 * Runs the auto steer loop until ball is in cargo, or timeed out
 * periodically plot positions to then reverse out?
 * Optional: Periodically then store the driven trajectory and drive it in reverse? 
 */
public class AutoSteerAction implements Action {
  private Arm mArm  = Arm.getInstance();
  private Drive mDrive = Drive.getInstance();
  private VisionManager mVisionManager = VisionManager.getInstance();
  private Grasper mGrasper = Grasper.getInstance();
  private boolean mComplete;
  private double mThrottleSpeed = .3;
  private boolean mAllowBacktrack;

  private enum Mode {
    VisionSteering, 
    Backtracking
  };
  private Mode mCurrentMode = Mode.VisionSteering;


  public AutoSteerAction(boolean allowBacktrack){
      mAllowBacktrack = allowBacktrack;
      mComplete = false;
  }

  @Override
  public void start() {


  }

  @Override
  public void update(){
    switch(mCurrentMode){
      case VisionSteering:
        TargetInfo ti = mVisionManager.getSelectedTarget(Constants.Vision.kAllowedSecondsThreshold);
        double errorAngle = 0;

        // check if ball is in grasper, or we can't see the ball anymore
        if(mGrasper.getBallsStored() > 0 || ti == null){
          // Done - either go backtrack or be done
          if(mAllowBacktrack){
            mCurrentMode = Mode.Backtracking;
          }else{
            // complete
            mComplete = true;
            mDrive.setDrive(0, 0, false);
          }
        }else{
          // continue driving
          mDrive.autoSteer(mThrottleSpeed, errorAngle);
        }
        break;
      case Backtracking:
        // now following back the path we took to auto steer

        break;
      default:
        break;
    }
  }
  /*
  @Override
  public void update() {
    TargetInfo ti = mVisionManager.getSelectedTarget(Constants.Vision.kAllowedSecondsThreshold);
    if(ti != null){
        // steer to the target
    }else{
        // no vision packet, must be done? or this is an error
        mComplete = true;
        return;
    }

    // check if ball is in grasper
    if(mGrasper.hasBall()){
      mComplete = true;
      return;
    }

    // command drive
    double errorAngle = 0;
    mDrive.autoSteer(mThrottleSpeed, errorAngle);


    
    // if allow backtrack, we should periodically store robot position
    if(mAllowBacktrack){
      
    }
  }
  */

  @Override
  public boolean isFinished() {
      return mComplete;
  }

  @Override
  public void done() {
    // stop driving
    mDrive.setDrive(0, 0, false);
  }
}
