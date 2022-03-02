package frc.robot.auto.actions;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmTarget;
import frc.robot.vision.*;
import edu.wpi.first.wpilibj.PowerDistribution;
/**
 * Uses Vision to turn to the ball
 */
public class AutoTurnAction implements Action {
  private Drive mDrive = Drive.getInstance();
  private VisionManager mVisionManager = VisionManager.getInstance();
  private boolean mComplete;

  public AutoTurnAction(){
    mComplete = false;
  }

  @Override
  public void start() {

  }

  @Override
  public void update(){
    TargetInfo ti = mVisionManager.getSelectedTarget(Constants.Vision.kAllowedSecondsThreshold);
    double errorAngle = 0;
    if(ti != null && ti.isValid()){
        // has valid error
        errorAngle = ti.getErrorAngle();
        System.out.println("error angle "+ errorAngle);
    }
    // turn in place to errorAngle
    mDrive.autoSteer(0, errorAngle);
    if(Math.abs(errorAngle) <= 4) {
      mComplete = true;
      System.out.println("Done!");
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
  }
}