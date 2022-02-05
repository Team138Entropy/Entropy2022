package frc.robot.auto.actions;

import frc.robot.auto.TrajectoryFollower;
import edu.wpi.first.math.trajectory.*;
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
// https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html
public class TurnInPlaceAction implements Action {
    private boolean mStopWhenDone;
    private boolean mComplete;
    private double mDegrees;
    private Drive mDrive = Drive.getInstance();
    private PIDController mPIDController;

    public TurnInPlaceAction(double degrees,boolean stopWhenDone) {
        mComplete = false;
        mStopWhenDone = stopWhenDone;
        mDegrees = degrees;

        mPIDController = new PIDController(.1, 0, 0);
        mPIDController.setSetpoint(degrees);
        mPIDController.setTolerance(1); // degrees tolerance
    }


    @Override
    public void start() {
        System.out.println("Target Degrees" + mDegrees);
    }

    @Override
    public void update() {
        double gyroAngle = mDrive.getGyro().getAngle();
        double out = mPIDController.calculate(gyroAngle);
        double clampedOutput = MathUtil.clamp(out, -1, 1);

        // square potentially?
        clampedOutput = Math.copySign(clampedOutput * clampedOutput, clampedOutput);


        if(mPIDController.atSetpoint()){
            // Stop Robot
            mDrive.setPercentOutputDrive(0, 0);
            mComplete = true;

        }else{
            // Turning
            mDrive.setPercentOutputDrive(clampedOutput, -1 * clampedOutput);
        }

        /*
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
          mDrive.setPercentOutputDrive(0,0);
          mComplete = true;   
        }
        */
    }

    // if trajectory is done
    @Override
    public boolean isFinished() {
        return mComplete;
    }

    @Override
    public void done() {
        mDrive.setPercentOutputDrive(0,0);
        System.out.println("Action: Turn in Place Complete");
    }
}