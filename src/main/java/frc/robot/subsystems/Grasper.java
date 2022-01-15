package frc.robot.subsystems;

public class Grasper extends Subsystem {
    private static Grasper mInstance;

    public enum intakeStatuses {
        INTAKING, // wheels are intaking
        EJECTING, // wheels are ejecting balls
        IDLE // wheels are not moving
    }
    public intakeStatuses intakeStatus;

    public static synchronized Grasper getInstance(){
        if (mInstance == null) {
            mInstance = new Grasper();
          }
        return mInstance;
    }
    
    private Grasper(){
    
    }

    public void enableIntakeWheels(){
        intakeStatus = intakeStatuses.INTAKING;
    }

    public void enableEjectWheels(){
        intakeStatus = intakeStatuses.EJECTING;
    }

    public void stopAll(){
        intakeStatus = intakeStatuses.IDLE;
    }

    public intakeStatuses getStatus(){
        return intakeStatus;
    }

    public void zeroSensors() {

    }

    public void checkSubsystem() {

    }
}
