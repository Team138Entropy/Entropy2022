package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Grasper extends Subsystem {
  private static Grasper mInstance;
  
  private static PWMTalonSRX mTalon;

  private IntakeStatus intakeStatus;
  private int ballsStored;
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
    mTalon = new PWMTalonSRX(Constants.Grasper.pwmChannel);

    intakeStatus = IntakeStatus.IDLE;
    ballsStored = 0;
    currentThreshold = 0;
    minThresholdExceedCount = 5;
    thresholdExceedCount = 0;
    pulseCounter = 0;
    pulseCounterTime = 40;
  }

  public void update(double current) {
    switch (intakeStatus) {
      case INTAKE:
        if (current > currentThreshold) {
          thresholdExceedCount++;
        } else {
          thresholdExceedCount = 0;
        }

        if (thresholdExceedCount > minThresholdExceedCount) {
          ballsStored++;
          currentThreshold = 0;
          minThresholdExceedCount = 5;
          intakeStatus = IntakeStatus.IDLE;
          pulseCounter = 0;
        }
      case EJECT:
        if (ballsStored == 0) {
          intakeStatus = IntakeStatus.IDLE;
          pulseCounter = 0;
        }
        ballsStored = 0;
        currentThreshold = 0;
        minThresholdExceedCount = 5;
      case IDLE:
        stop();

        pulseCounter++;
        if (pulseCounter > pulseCounterTime) {
          mTalon.set(Constants.Grasper.jogSpeed);
          pulseCounter = 0;
        }

        if (pulseCounter == 5) stop();
      default:
        System.out.println("Grasper has no state!");
    }
  }

  public void intake(){
    mTalon.set(Constants.Grasper.jogSpeed);
    intakeStatus = IntakeStatus.INTAKE;
  }

  public void eject(){
    mTalon.set(-Constants.Grasper.jogSpeed);
    intakeStatus = IntakeStatus.EJECT;
    ballsStored = 1;
  }

  public void stop(){
    mTalon.set(0);
    intakeStatus = IntakeStatus.IDLE;
  }

  public IntakeStatus getStatus(){
    return intakeStatus;
  }

  public int getBallsStored() {
    return ballsStored;
  }

  public IntakeStatus getIntakeStatus() {
    return intakeStatus;
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}

  @Override
  public void updateSmartDashBoard(){
    SmartDashboard.putNumber("ballsStored", getBallsStored());
    SmartDashboard.putString("intakeStatus", getIntakeStatus().toString());
  }
}
