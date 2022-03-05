package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.DutyCycleExecutor;
import java.util.concurrent.Callable;
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
  private int mPulseCounterTime; // How long to wait between pulses, measured in robot loops
  private int mMaxPulseTime;
  private int mTimeSinceStart;
  private int mStartWaitTime;

  // Pulse Exector (On for X amount of Seconds, Off for X amount of Seconds)
  private final double mPulseOnSeconds = .2;
  private final double mPulseOffSeconds = 2;
  private DutyCycleExecutor mPulseExecutor = new DutyCycleExecutor(
      new Callable<Boolean>() {
          public Boolean call(){
              // on function
              // turn on grasper
              mTalon.set(kJogSpeed);
              return true;
          }
      },
      new Callable<Boolean>() {
        public Boolean call(){
            // off function
            // turn off grasper
            stop();
            return true;
        }
    },
    mPulseOnSeconds,
    mPulseOffSeconds
  );

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
    mCurrentThreshold = 10;
    mMinThresholdExceedCount = 6;
    mThresholdExceedCount = 0;
    mPulseCounter = 0;
    mPulseCounterTime = 10;
    mMaxPulseTime = 60;
    mTimeSinceStart = 0;
    mStartWaitTime = 10;
  }

  public void update(double current) {
    switch (mIntakeStatus) {
      case INTAKE:
      System.out.println("GRASPER: INTAKE! "+ current);
      mTimeSinceStart++;
        if (mBallsStored >= Constants.Grasper.maxBallsStored) {
          System.out.println("GRASPER STOP!");
          stop();
          mIntakeStatus = IntakeStatus.IDLE;
        }

        // If the current exceeds the normal level then we have a ball
        if (current > mCurrentThreshold && mTimeSinceStart > mStartWaitTime) {
          mThresholdExceedCount++;
          System.out.println(" OVER THREADSHOLD!");
        } else if (mTimeSinceStart <= mStartWaitTime) {
          mThresholdExceedCount = 0;
          System.out.println("ZEROING THRESHOLD!");
        }

        // If the current exceeds a set value for a set time and stop intaking
        if (mThresholdExceedCount > mMinThresholdExceedCount) {
          stop();
          mBallsStored++;
          mIntakeStatus = IntakeStatus.IDLE;
          mPulseCounter = 0;
          mPulseExecutor.reset();
        }
        break;
      case EJECT:
      System.out.println("GRAPSER: EJECT!");
        mThresholdExceedCount = 0;
        mTimeSinceStart = 0;
        if (mBallsStored == 0) {
          stop();
          mIntakeStatus = IntakeStatus.IDLE;
          mPulseCounter = 0;
        }
        mBallsStored = 0;
        mPulseExecutor.reset();
        break;
      case IDLE:
      System.out.println("GRASPER: IDLE");
        mThresholdExceedCount = 0;
        mTimeSinceStart = 0;

        // The motor will periodically run to make sure that the ball is secure and hasn't came loose
        if(mBallsStored > 0){
          mPulseExecutor.update();
        }
        break;
      default:
        System.out.println("Grasper has no status!");
    }
    SmartDashboard.putNumber("Graspercurrent", current);
  }

  public void intake() {
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
    SmartDashboard.putNumber("Exceed Count", mThresholdExceedCount);
  }
}
