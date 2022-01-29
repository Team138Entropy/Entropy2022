package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import frc.robot.Constants;

public class Grasper extends Subsystem {
    private static Grasper mInstance;
    
    private static PWMTalonSRX mTalon;

    private IntakeStatus intakeStatus;
    private boolean hasBall;
    private double currentThreshold;
    private double minThresholdExceedCount; // How many consecutive loops the current must exceed the threshold
    private double thresholdExceedCount; // How many consecutive loops the current has exceeded the threshold for
    private double pulseCounter;
    private double pulseCounterTime;

    public enum IntakeStatus {
        INTAKE, // Wheels are intaking
        EJECT, // Wheels are ejecting balls
        IDLE // Wheels are not moving
    }

    public static synchronized Grasper getInstance(){
        if (mInstance == null) {
            mInstance = new Grasper();
          }
        return mInstance;
    }
    
    private Grasper(){
        mTalon = new PWMTalonSRX(0);

        intakeStatus = IntakeStatus.IDLE;
        currentThreshold = 0;
        minThresholdExceedCount = 5;
        thresholdExceedCount = 0;
        pulseCounter = 0;
        pulseCounterTime = 100;
    }

    public void update(double current) {
        if (thresholdExceedCount > minThresholdExceedCount && current > currentThreshold) {
            hasBall = true;
            intakeStatus = IntakeStatus.IDLE;
            pulseCounter = 0;
        } else if (current > currentThreshold) {
            thresholdExceedCount++;
        } else {
            thresholdExceedCount = 0;
            hasBall = false;
            intakeStatus = IntakeStatus.IDLE;
        }

        switch (intakeStatus) {
            case INTAKE:
                
            case EJECT:
                
            case IDLE:
                stop();
                if (pulseCounter > pulseCounterTime) {
                    mTalon.set(Constants.Grasper.jogSpeed);
                    pulseCounter = 0;
                }
        
                if (pulseCounter == 5) stop();
                
                pulseCounter++;
            default:
                System.out.println("Grasper has no state");
        }
    }

    public synchronized void intake(){
        intakeStatus = IntakeStatus.INTAKE;
        mTalon.set(Constants.Grasper.jogSpeed);
    }

    public synchronized void eject(){
        intakeStatus = IntakeStatus.EJECT;
        mTalon.set(-Constants.Grasper.jogSpeed);
    }

    public synchronized void stop(){
        intakeStatus = IntakeStatus.IDLE;
        mTalon.set(0);
    }

    public IntakeStatus getStatus(){
        return intakeStatus;
    }

    public boolean hasBall() {
        return hasBall;
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void checkSubsystem() {

    }

    @Override
    public void updateSmartDashBoard(){

    }
}
