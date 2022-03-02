package frc.robot.auto.actions;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmTarget;
import edu.wpi.first.wpilibj.Timer;

/*
    Drive Until Pickup Action
    Keeps Encoder Straight
*/
public class DriveUntilPickupAction implements Action {
    private Arm mArm  = Arm.getInstance();
    private Drive mDrive = Drive.getInstance();
    private Grasper mGrasper = Grasper.getInstance();
    private boolean mComplete;
    private double mThrottleSpeed = -.16;
    private double mStartingEncoderAngle;
    private Timer mTimer;
    private double mTimeoutSeconds = 8;
    private int mContinueDrivingTime = 3;
    private int mDriveTime = 0;

    public DriveUntilPickupAction(){
        mComplete = false;
        mTimer = new Timer();
    }

    @Override
    public void start() {
      mTimer.start();
      mStartingEncoderAngle = mDrive.getGyro().getAngle();
      mArm.rotateToPosition(ArmTarget.INTAKE.degrees);
      mGrasper.intake();
    }

    @Override 
    public void update(){
        System.out.println("in update loop");
        mGrasper.update(Constants.Grasper.globelPowerDistribution.getCurrent(Constants.Grasper.powerDistributionNumber));

        if(mGrasper.getBallsStored() > 0){
            System.out.println("got the ball");
            // Ball is stored!
            
           mDriveTime++;
           if (mDriveTime > mContinueDrivingTime) {
            mComplete = true;
           mDrive.setDrive(0, 0, false);
            mArm.rotateToPosition(ArmTarget.SCORE_FRONT.degrees);
           }
        }else{
            double currentErrorAngle = mDrive.getGyro().getAngle() - mStartingEncoderAngle;
            mDrive.autoSteer(mThrottleSpeed, 0);
        }
        
        // check if timeout
        if(mTimer.hasElapsed(mTimeoutSeconds)) {
            System.out.println("time out");
            mComplete = true;
            mGrasper.stop();
            mDrive.setDrive(0, 0, false);
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
      System.out.println("stop drving");
    }
}
