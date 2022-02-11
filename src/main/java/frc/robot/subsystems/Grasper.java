package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Claw like appendage that can acquire and store balls, and climb.
 */
public class Grasper extends Subsystem {
  private static Grasper mInstance;
  
  private final double kJogSpeed = .8;

  private PWMTalonSRX mTalon;
  
  private IntakeStatus mIntakeStatus;
  private int mBallsStored;
  private double mCurrentThreshold; // The threshold the current must exceed to register
  private int mThresholdExceedCount; // How many consecutive loops the current has exceeded the threshold for
  private int mMinThresholdExceedCount; // How many consecutive loops the current must exceed the threshold
  private int mPulseCounter; // Counts how long its been since the last "pulse" (running the motor)
  private double mPulseCounterTime; // How long to wait between pulses, measured in robot loops

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

    mIntakeStatus = IntakeStatus.IDLE;
    mThresholdExceedCount = 0;
    mBallsStored = 0;
    mCurrentThreshold = 6;
    mMinThresholdExceedCount = 30;
    mThresholdExceedCount = 0;
    mPulseCounter = 0;
    mPulseCounterTime = 40;
  }

  public void update(double current) {
    System.out.println(current);
    switch (mIntakeStatus) {
      case INTAKE:
        System.out.println("exceeding count" + mThresholdExceedCount);
        System.out.println("Intaking");
        if (mBallsStored >= Constants.Grasper.maxBallsStored) {
          stop();
          mIntakeStatus = IntakeStatus.IDLE;
        }

        // If the current exceeds the normal level then we have a ball
        if (current > mCurrentThreshold) {
          mThresholdExceedCount++;
          System.out.println("Exceeding");
        } else {
          mThresholdExceedCount = 0;
          System.out.println("Reset to zero");
        }

        // If the current exceeds a set value for a set time and stop intaking
       System.out.println(mThresholdExceedCount + " > " +  mMinThresholdExceedCount);
        if (mThresholdExceedCount > mMinThresholdExceedCount) {
          System.out.println("Finished intaking");
          stop();
          mBallsStored++;
          mCurrentThreshold = 0;
          mIntakeStatus = IntakeStatus.IDLE;
          mPulseCounter = 0;
        }
        break;
      case EJECT:
        System.out.println("Ejecting");
        if (mBallsStored == 0) {
          System.out.println("Finished");
          stop();
          mIntakeStatus = IntakeStatus.IDLE;
          mPulseCounter = 0;
        }
        mBallsStored = 0;
        mCurrentThreshold = 0;
        break;
      case IDLE:
        System.out.println("Idle");

        // The motor will periodically run to make sure that the ball is secure and hasn't came loose
        mPulseCounter++;
        // if (mPulseCounter > mPulseCounterTime) {
        //   System.out.println("Pulsing");
        //   mTalon.set(kJogSpeed);
        //   mPulseCounter = 0;
        // }

        // Stop after 5 loops
        if (mPulseCounter == 5) stop();
        break;
      default:
        System.out.println("Grasper has no state!");
    }
  }

  public void intake(){
    mTalon.set(kJogSpeed);
    mIntakeStatus = IntakeStatus.INTAKE;
  }

  public void eject(){
    mTalon.set(-kJogSpeed);
    mIntakeStatus = IntakeStatus.EJECT;
    mBallsStored = 1;
  }

  public void stop(){
    mTalon.set(0);
    mIntakeStatus = IntakeStatus.IDLE;
  }

  public IntakeStatus getStatus(){
    return mIntakeStatus;
  }

  public int getBallsStored() {
    return mBallsStored;
  }

  public IntakeStatus getIntakeStatus() {
    return mIntakeStatus;
  }

  public void intakeManual() {
    mTalon.set(kJogSpeed);
    mIntakeStatus = null;
  }

  public void ejectManual() {
    mTalon.set(-kJogSpeed);
    mIntakeStatus = null;
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {}

  @Override
  public void updateSmartDashBoard(){
    SmartDashboard.putNumber("ballsStored", getBallsStored());
    if(getIntakeStatus() != null){
      SmartDashboard.putString("intakeStatus", getIntakeStatus().toString());
    }
    SmartDashboard.putNumber("Grasper Percent Output", mTalon.get());
  }
}
