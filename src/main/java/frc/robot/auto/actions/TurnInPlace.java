package frc.robot.auto.actions;

import frc.robot.auto.TrajectoryFollower;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.controller.PIDController;
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
    private double mDagrees;
    private Drive mDrive=Drive.getInstance();
    private PIDController mPidController;

    public TurnInPlace(double drgrees,boolean stopWhenDone) {
        mStopWhenDone = stopWhenDone;
        mDagrees = drgrees;
        mPidController = new PIDController(0.2, 0, 0.015);
        mPidController.setTolerance(1);
        mPidController.setSetpoint(drgrees);
    }


    @Override
    public void start() {
        System.out.println("Target drgrees"+mDagrees);
        mDrive.getGyro().reset();
    }

    @Override
    public void update() {
       double gyroAngle = mDrive.getGyro().getAngle();
       double output = mPidController.calculate(gyroAngle);
       System.out.println("Gyro Angle: " + gyroAngle);
       System.out.println("Ouput: " + output);
       mDrive.setDrive(0, output, false);
    }

    // if trajectory is done
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {
        System.out.println("turn in place complete");
    }
}