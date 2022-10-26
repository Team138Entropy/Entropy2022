package frc.robot.auto.actions;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmTarget;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
    Pickup Action
    Stops or Times out after a certain amount of time
*/
public class PickupAction implements Action {
    private Arm mArm  = Arm.getInstance();
    private Grasper mGrasper = Grasper.getInstance();
    private boolean mComplete;
    private Timer mTimer;
    private double mTimeoutSeconds = 6;

    public PickupAction(){
        mComplete = false;
        mTimer = new Timer();
    }

    public PickupAction(double timeoutTime){
        mComplete = false;
        mTimer = new Timer();
        mTimeoutSeconds = timeoutTime;
    }

    @Override
    public void start() {
      System.out.println("PickupAction::Start");
      mTimer.start();
      mArm.rotateToPosition(ArmTarget.SCORE_FRONT.degrees);
      mGrasper.intake();
    }

    @Override 
    public void update(){
        mGrasper.update(Constants.Grasper.globelPowerDistribution.getCurrent(Constants.Grasper.powerDistributionNumber));
        if(mGrasper.getBallsStored() > 0){
            System.out.println("PickupAction::Ball Retrieved");
            // Ball is stored!
            mComplete = true;
            mGrasper.stop();
            mArm.rotateToPosition(ArmTarget.HOME.degrees);
        }

        // check if timeout
        if(mTimer.hasElapsed(mTimeoutSeconds)) {
            System.out.println("PickupAction:Timeout");
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
      System.out.println("PickupAction::Done");
    }
}
