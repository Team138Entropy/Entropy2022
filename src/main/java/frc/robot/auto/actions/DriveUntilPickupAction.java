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
    private double mThrottleSpeed = -.19;
    private double mStartingEncoderAngle;
    private Timer mTimer;
    private double mTimeoutSeconds = 8;

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
        double currentErrorAngle = mDrive.getGyro().getAngle() - mStartingEncoderAngle;
        mDrive.setDrive(mThrottleSpeed, currentErrorAngle, true);
        mGrasper.update(Constants.Grasper.globelPowerDistribution.getCurrent(Constants.Grasper.powerDistributionNumber));
        if(mGrasper.getBallsStored() > 0){
            // Ball is stored!
            mComplete = true;
        }
        
        // check if timeout
        if(mTimer.hasElapsed(mTimeoutSeconds)) {
            mComplete = true;
            mGrasper.stop();
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
