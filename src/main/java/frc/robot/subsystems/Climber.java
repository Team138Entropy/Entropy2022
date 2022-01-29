package frc.robot.subsystems;

/**
 * Climber is intended to be mostly autonomous
 * Stage 1 - 
 *        Climber is raising to climbing position
 *        Arm is in low position
 *        Complete when arm is at full extension
 * Stage 2 -
 *        Climber is pulling up to top bar
 *        Arm is in low position
 *        Complete when climber is at top
 */
public class Climber extends Subsystem {
    private static Climber mInstance;

    public enum ClimberStage {

    };

    public static synchronized Climber getInstance(){
        if (mInstance == null) {
            mInstance = new Climber();
          }
        return mInstance;
    }
    
    private Climber(){
    
    }

    public void zeroSensors() {

    }

    public void checkSubsystem() {

    }

    public void updateSmartDashBoard() {

    }
}
