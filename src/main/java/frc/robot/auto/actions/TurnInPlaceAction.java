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

    public TurnInPlaceAction(double degrees) {
        mComplete = false;
        mDegrees = degrees;
    }


    @Override
    public void start() {
        System.out.println("TurnInPlaceAction - Target Degrees" + mDegrees);
        mDrive.getGyro().reset();
    }

    @Override
    public void update() {
        System.out.println("TurnInPlaceAction - Update");
        double angle = mDegrees - mDrive.getGyro().getAngle();
        System.out.println("  angle: " + angle);
        mDrive.autoSteer(.16, angle);   
        if(Math.abs(angle) < 2){
            // within degrees 
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
        mDrive.setPercentOutputDrive(0,0);
        System.out.println("Action: Turn in Place Complete");
    }
}