package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import frc.robot.Constants;

public class Grasper extends Subsystem {
    private static Grasper mInstance;
    
    private static PWMTalonSRX mTalon;

    private IntakeStatus intakeStatus;
    private boolean hasBall;
    private double currentThreshold;
    private double averageCurrent;

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
        averageCurrent = 0;

    }

    public void update(double current) {
        averageCurrent += (current - averageCurrent) / 5;

        if (averageCurrent > currentThreshold) {
            hasBall = true;
        } else {
            hasBall = false;
        }

        switch (intakeStatus) {
            case INTAKE:
                if (hasBall) stop();
            case EJECT:
                if (!hasBall) stop();
            case IDLE:
                stop();
            default:
                System.out.println("Grasper has no state");
        }
    }

    public void intake(){
        intakeStatus = IntakeStatus.INTAKE;
        mTalon.set(Constants.Grasper.jogSpeed);
    }

    public void eject(){
        intakeStatus = IntakeStatus.EJECT;
        mTalon.set(-Constants.Grasper.jogSpeed);
    }

    public void stop(){
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
}
