package frc.robot.util;

import java.util.concurrent.Callable;
import edu.wpi.first.wpilibj.Timer;

/*
    DutyCycle Executor
    Goes Off or On and calls a callable
*/
public class DutyCycleExecutor {
    private Callable<Boolean> mOnFunction;
    private Callable<Boolean> mOffFunction;
    private double mOnSeconds;
    private double mOffSeconds;
    private boolean mIsOn;
    private Timer mTimer;

    public DutyCycleExecutor(Callable<Boolean> onFunc, Callable<Boolean> offFunc, double onSeconds, double offSeconds){
        mOnFunction = onFunc;
        mOffFunction = offFunc;
        mOnSeconds = onSeconds;
        mOffSeconds = offSeconds;
        mIsOn = false;
        mTimer = null;
    }

    public synchronized void update(){
        // Create and Start Timer
        if(mTimer == null){
            mTimer = new Timer();
            mTimer.start();
        }

        if(mIsOn){
            // On!
            if(!mTimer.hasElapsed(mOnSeconds)){
                // still in off period
                try{
                    mOnFunction.call();
                } catch (Exception e){
                    // Exception calling callable
                }
            }else{
                // back to on period
                mTimer = null;
                mIsOn = false;
            }
        }else{
            // Off!
            if(!mTimer.hasElapsed(mOffSeconds)){
                // still in off period
                try{
                    mOffFunction.call();
                } catch (Exception e){
                    // Exception calling callable
                }
            }else{
                // back to on period
                mTimer = null;
                mIsOn = true;
            }
        }
    }

    // set back to off
    public synchronized void reset(){
        mIsOn = false;
        mTimer = null;
    }

    public synchronized void setOn(){
        mTimer = null;
        mIsOn = true;
    }

    public synchronized void setOff(){
        mTimer = null;
        mIsOn = false;
    }

    public synchronized boolean isOn(){
        return mIsOn;
    }
}
