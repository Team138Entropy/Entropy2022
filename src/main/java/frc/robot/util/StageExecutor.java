package frc.robot.util;

import java.util.Vector;
import java.util.concurrent.Callable;

class Stage {
    public final Callable<Boolean> mStageWorkerFunction;
    public final Callable<Boolean> mStageCompleteFunction;
    public final String mName;
    public final boolean mRequireUserToStart;

    public Stage(String Name, Callable<Boolean> workerFunc, Callable<Boolean> completeFunc){
        mName = Name;
        mStageWorkerFunction = workerFunc;
        mStageCompleteFunction = completeFunc;
        mRequireUserToStart = false;
    }

    public Stage(String Name, Callable<Boolean> workerFunc, Callable<Boolean> completeFunc, boolean reqStart){
        mName = Name;
        mStageWorkerFunction = workerFunc;
        mStageCompleteFunction = completeFunc;
        mRequireUserToStart = reqStart;
    }

    public Stage(Callable<Boolean> workerFunc, Callable<Boolean> completeFunc){
        mName = "";
        mStageWorkerFunction = workerFunc;
        mStageCompleteFunction = completeFunc;
        mRequireUserToStart = false;
    }

    public Stage(Callable<Boolean> workerFunc, Callable<Boolean> completeFunc, boolean reqStart){
        mName = "";
        mStageWorkerFunction = workerFunc;
        mStageCompleteFunction = completeFunc;
        mRequireUserToStart = reqStart;
    }

    public boolean hasName(){
        return mName.length() > 0;
    }
};

/* Executes a Stage, Checks conditions and then goes to the next stage
 * 
 */
public class StageExecutor {
    
    private int mCurrentStage;
    private int mCurrentStageLoopCount;
    private String mCurrentStageName;
    private boolean mVerboseMode;
    private boolean mRunning;
    private boolean mNeedUserInputToStartStage;
    private boolean mStartBlessed;
    private Vector<Stage> mStages;

    public StageExecutor(){
        mCurrentStage = 0;
        mCurrentStageLoopCount = 0;
        mVerboseMode = false;
        mRunning = false;
        mNeedUserInputToStartStage = false;
        mStartBlessed = false;
        mCurrentStageName = "";
        mStages = new Vector<Stage>();
    }

    // Adds a New Stage to be Executed
    // StageWorkerFunction is called every loop while in a stage
    public synchronized void registerStage(Callable<Boolean> stageWorkerFunction, Callable<Boolean> stageCompleteFunction){
        mStages.add(new Stage(stageWorkerFunction, stageCompleteFunction));
    }

    public synchronized void registerStage(Callable<Boolean> stageWorkerFunction, Callable<Boolean> stageCompleteFunction, boolean ReqUserInputStart){
        mStages.add(new Stage(stageWorkerFunction, stageCompleteFunction, ReqUserInputStart));
    }

    // Adds a New Stage to be Executed
    public synchronized void registerStage(String Name, Callable<Boolean> stageWorkerFunction, Callable<Boolean> stageCompleteFunction){
        mStages.add(new Stage(Name, stageWorkerFunction, stageCompleteFunction));
    }

    public synchronized void registerStage(String Name, Callable<Boolean> stageWorkerFunction, Callable<Boolean> stageCompleteFunction, boolean ReqUserInputStart){
        mStages.add(new Stage(Name, stageWorkerFunction, stageCompleteFunction, ReqUserInputStart));
    }

    // Updates the Stage Executor
    public synchronized boolean update(){
        return update(false);
    }

    // Updates the Stage Executor
    // Allows a User Input to Approve Next Stage (if Stage Requires)
    // returns true when complete
    public synchronized boolean update(boolean userInput){
        boolean result = false;
        if(mCurrentStage < mStages.size()){
            // still at a valid stage
            Stage currentStage = mStages.get(mCurrentStage);
            mCurrentStageLoopCount++;
            mCurrentStageName = currentStage.mName;
            
            // first loop, store that we need to get user input
            if(mCurrentStageLoopCount == 1){
                mNeedUserInputToStartStage = currentStage.mRequireUserToStart;
            }
            if(mVerboseMode) System.out.println("Executing Stage: " + mCurrentStage);
            try {
                // check if we need blessing to start
                if(mNeedUserInputToStartStage && !mStartBlessed){
                    if(mVerboseMode) System.out.println("Waiting for User Input to Start Stage: " + mCurrentStage);
                    if(userInput){
                        // user says start
                        mStartBlessed = true;
                        mNeedUserInputToStartStage = false;
                    }else{
                        // no start blessing, go to next loop
                        return false;
                    }
                }

                // Executing Stage
                currentStage.mStageWorkerFunction.call();
            }catch(Exception e){
                // Exception Caught
                System.out.println("StageExecutor Worker Exception!");
                System.out.println(e.getMessage());
            }
            
            
            // Check if Stage is Complete
            try{
                if(currentStage.mStageCompleteFunction.call()){
                    // Move to Next Stage
                    if(mVerboseMode) System.out.println("Stage " + mCurrentStage + " Complete!");
                    mCurrentStage++;
                    mCurrentStageLoopCount = 0; //clear out loop count
                    mStartBlessed = false;
                    mNeedUserInputToStartStage = false;
                }
            }catch(Exception e){
                // exception
                System.out.println("StageExecutor Complete Exception!");
                System.out.println(e.getMessage());                
            }    
            
        }else{
            // Stages Complete
            result = true;
        }
        return result;
    }

    public synchronized void reset(){
        mCurrentStage = 0;
        mCurrentStageLoopCount = 0; //clear out loop count
        mStartBlessed = false;
        mNeedUserInputToStartStage = false;
    }

    public synchronized void setVerboseMode() { mVerboseMode = true;}
    public synchronized void disableVerboseMode() { mVerboseMode = false;}

    // Stages are 1 Based, there is no stage 0
    public synchronized int getCurrentStage(){
        return mCurrentStage + 1;
    }
    
    public synchronized String getCurrentStageName(){
        return mCurrentStageName;
    }

    public synchronized boolean isComplete(){
        return mCurrentStage == mStages.size();
    }

    public synchronized boolean needUserInputToStart(){
        return mNeedUserInputToStartStage;
    }
}
