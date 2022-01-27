package frc.robot.subsystems;

;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends Subsystem {
    private static Climber mInstance;

    public enum ClimberStage {
        Stage1,
        Stage2,
        Stage3,
        Stage4
    };

    // Convert CLimber Stage to String
    public static String getClimberStageToString(ClimberStage value){
        switch(value){
            case Stage1: 
                return "Stage 1";
            case Stage2:
                return "Stage 2";
            case Stage3:
                return "Stage 3";
            case Stage4: 
                return "Stage 4";
            default: 
                return "";
        }
    }

    private ClimberStage mCurrentStage = ClimberStage.Stage1;

    public static synchronized Climber getInstance(){
        if (mInstance == null) {
            mInstance = new Climber();
          }
        return mInstance;
    }

    //private final Arm mArm = Arm
    
    private Climber(){
    
    }

    /**
     * Update Loop for the Climber
     * Likely be updated to take button presses
     */
    public synchronized void update(){
        switch(mCurrentStage){
            case Stage1:
                // Robot is below bar, extending arms to go up
                
            break;
            case Stage2: 

            break;
            case Stage3: 

            break;
            case Stage4: 

            break;
            default:
                System.out.println("Error: Unreconizied Climber Stage!");
                break;
        }
    }

    public void zeroSensors() {

    }

    public void checkSubsystem() {

    }

    public void updateSmartDashBoard() {
        
        SmartDashboard.putString("Climber Stage", getClimberStageToString(mCurrentStage));
    }
}
