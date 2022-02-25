package frc.robot.auto.actions;

import java.lang.annotation.Target;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmTarget;
import frc.robot.vision.*;
import edu.wpi.first.wpilibj.PowerDistribution;
/**
 * Uses Vision System to steer to a ball
 * Runs the auto steer loop until ball is in cargo, or timeed out
 * periodically plot positions to then reverse out?
 * Optional: Periodically then store the driven trajectory and drive it in reverse? 
 */
public class AutoTurn implements Action {
  private Arm mArm  = Arm.getInstance();
  private Drive mDrive = Drive.getInstance();
  private VisionManager mVisionManager = VisionManager.getInstance();
  private Grasper mGrasper = Grasper.getInstance();
  private boolean mComplete;
  private double mThrottleSpeed = -0.15;
  private boolean mAllowBacktrack;

  private enum Mode {
    VisionSteering, 
    Backtracking
  };
  private Mode mCurrentMode = Mode.VisionSteering;


  public AutoTurn(boolean allowBacktrack){
      mAllowBacktrack = allowBacktrack;
      mComplete = false;
  }

  @Override
  public void start() {
    mArm.rotateToPosition(ArmTarget.HOME.degrees);

  }

  @Override
  public void update(){
    TargetInfo ti = mVisionManager.getSelectedTarget(Constants.Vision.kAllowedSecondsThreshold);
    if(ti != null){
        // steer to the target
        double error = ti.getErrorAngle();
        if (Math.abs(error) <= 5) {
          mComplete = true;
        }
        else {
          mDrive.autoSteer(0, error);
        }
    }else{
        // no vision packet, must be done? or this is an error
        mComplete = true;
    }
  }

  @Override
  public boolean isFinished() {
      return mComplete;
  }

  @Override
  public void done() {
    // stop driving
    mDrive.setDrive(0, 0, false);
    System.out.println("I AM DONE !!!!!!!!!!!!!!!!!!!!!!!");
  }
}
