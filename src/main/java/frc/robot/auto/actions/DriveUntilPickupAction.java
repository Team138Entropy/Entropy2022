package frc.robot.auto.actions;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmTarget;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
    Drive Until Pickup Action
    Keeps Encoder Straight
*/
public class DriveUntilPickupAction implements Action {
    private Arm mArm  = Arm.getInstance();
    private Drive mDrive = Drive.getInstance();
    private Grasper mGrasper = Grasper.getInstance();
    private boolean mComplete;
    private double mThrottleSpeed = -.18;
    private double mStartingGyroErrorAngle;
    private Timer mTimer;
    private double mTimeoutSeconds = 6;
    private int mContinueDrivingTime = 0;
    private int mDriveTime = 10;

    public DriveUntilPickupAction(){
        mComplete = false;
        mTimer = new Timer();
    }

    @Override
    public void start() {
        System.out.println("drive until pickup start!");
      mTimer.start();
      mStartingGyroErrorAngle = mDrive.getGyro().getAngle();
      mArm.rotateToPosition(ArmTarget.INTAKE.degrees);
      mGrasper.intake();
    }

    @Override 
    public void update(){
        mGrasper.update(Constants.Grasper.globelPowerDistribution.getCurrent(Constants.Grasper.powerDistributionNumber));
        //System.out.println("DRIVE DRIVE DRIVE!");
        if(mGrasper.getBallsStored() > 0){
            System.out.println("got the ball");
            // Ball is stored!
            
           mDriveTime++;
           if (mDriveTime > mContinueDrivingTime) {
            mComplete = true;
            mDrive.updateOdometry();
            mDrive.setDrive(0, 0, false);
           }
        }else{
            System.out.println("drive drive drive!");
            // continue driving forward
            // TODO: This might be the wrong offset 
            double offsetGyro = mStartingGyroErrorAngle - mDrive.getGyro().getAngle();
            SmartDashboard.putNumber("Gyro Straight Offset", offsetGyro);
            mDrive.driveErrorAngle(mThrottleSpeed, offsetGyro);
            //mDrive.driveGyroSetpoint(mThrottleSpeed, mStartingGyroErrorAngle);

            mDrive.updateOdometry();
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
