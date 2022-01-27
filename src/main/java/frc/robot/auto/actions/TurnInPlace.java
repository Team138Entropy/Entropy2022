package frc.robot.auto.actions;

import frc.robot.auto.TrajectoryFollower;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;
/**
 * Drives the Trajectory using the Trajectory Follower
 * @see PathContainer
 * @see Path
 * @see Action
 */
public class TurnInPlace implements Action {
    private boolean mStopWhenDone;
    private boolean mComplete;
    private double mDegrees;
    private Drive mDrive=Drive.getInstance();

    public TurnInPlace(double degrees,boolean stopWhenDone) {
        mComplete = false;
        mStopWhenDone = stopWhenDone;
        mDegrees = degrees;
    }


    @Override
    public void start() {
        System.out.println("Target drgrees" + mDegrees);
        mDrive.getGyro().reset();
    }

    @Override
    public void update() {
        double gyroAngle = mDrive.getGyro().getAngle();
        final double kP = 0.005;
        
        if (mDegrees > 180) {
            mDegrees = -(360 - mDegrees);
        } else if (mDegrees < -180) {
            mDegrees = 360 + mDegrees;
        }
        
        double err = mDegrees - gyroAngle;
        double speed = MathUtil.clamp(err * kP, -0.4, 0.4);
        
        if(Math.abs(err) > 2){
          mDrive.setDrive(0, speed, true);   
        }else{
          mDrive.setDrive(0,0,false);
          mComplete = true;   
        }
    }

    // if trajectory is done
    @Override
    public boolean isFinished() {
        return mComplete;
    }

    @Override
    public void done() {
        mDrive.setDrive(0,0,false);
        System.out.println("turn in place complete");
    }
}
