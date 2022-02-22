package frc.robot.auto.actions;

import frc.robot.subsystems.Grasper;

public class EjectAction implements Action {
    private final Grasper mGrasper = Grasper.getInstance();
    private final int mMotorRunTime = 20;
    private int mLoopCount;

    public EjectAction() {
        mLoopCount = 0;
    }

    @Override
    public void start() {}

    @Override
    public void update() {
        mGrasper.eject();
        mLoopCount++;
    }

    @Override
    public boolean isFinished() {
        return mLoopCount >= mMotorRunTime;
    }

    @Override
    public void done() {
        mGrasper.stop();
    }   
}
