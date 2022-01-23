package frc.robot.subsystems;

public class Climber extends Subsystem {
    private static Climber mInstance;

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
