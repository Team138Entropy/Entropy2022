package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import frc.robot.Constants;

/**
 * Arm subsystem comprised of a rotating shoulder and telescoping extender.
 */
public class Arm extends Subsystem {
  private static Arm mInstance;

  private TalonSRX mShoulder;
  private Talon mForearm;
  private Encoder mForearmEncoder;

  double kTurnTravelUnitsPerRotation = 360;
  double kEncoderUnitsPerRotation = 8192.0;

  public Arm() {
    mShoulder = new TalonSRX(5);
    mForearm = new Talon(Constants.Arm.forearmChannel);
    mForearmEncoder = new Encoder(0, 1);

    mShoulder.setSensorPhase(true);
    mShoulder.setSelectedSensorPosition(60);
    System.out.println("Starting Position: " + getShoulderPosition());
    // Configure Sensor Feedback
    mShoulder.configSelectedFeedbackCoefficient(kTurnTravelUnitsPerRotation / kEncoderUnitsPerRotation);

    mShoulder.config_kP(0, 20, 10);
		mShoulder.config_kI(0, 0, 10);
		mShoulder.config_kD(0, .25, 10);

    //mShoulder.ConfigSelectedFeedbackCoefficient(kTurnTravelUnitsPerRotation / kEncoderUnitsPerRotation, 1, 10);
  }

  public void testRotate(){
    int kMeasuredPosHorizontal = 840; //Position measured when arm is horizontal
    double kTicksPerDegree = 8192 / 360; //Sensor is 1:1 with arm rotation


    /*
    double degrees = (currentPos) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    */

    int currentPos = mShoulder.getSelectedSensorPosition();
    double radians = java.lang.Math.toRadians(currentPos);
    double cos = java.lang.Math.cos(radians);
    //double absCos = java.lang.Math.abs(cos);

    double maxGravityFF = 0.5;
    double feedforward = maxGravityFF * cos;
    System.out.println("FeedForward: " + feedforward);
    int targetPos = 0;
    feedforward = 0;
    mShoulder.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, feedforward);
  }


  public static Arm getInstance() {
    if (mInstance == null) {
      mInstance = new Arm();
    }
    return mInstance;
  }

  public void rotate(double speed) {
    //speed = Math.max(Math.min(speed, 1), -1);
    mShoulder.set(ControlMode.PercentOutput, speed);
  }

  public void rotateShoulderDistance(double degrees) {
    degrees = Math.max(Math.min(degrees, (Constants.Arm.maxEncoderPositionShoulder - getShoulderPosition()) / Constants.Arm.ticksPerDegreeShoulder), -getShoulderPosition() / Constants.Arm.ticksPerDegreeShoulder);
    mShoulder.set(ControlMode.MotionMagic, getShoulderPosition() + degrees * Constants.Arm.ticksPerDegreeShoulder, DemandType.ArbitraryFeedForward, getGravityFeedForward());
  }

  private double getGravityFeedForward() {
    int currentPos = getShoulderPosition() - Constants.Arm.positionHorizontal;
    double currentRads = currentPos * Constants.Arm.ticksPerRadianShoulder;
    double ff = Constants.Arm.maxGravityFF * Math.cos(currentRads);
    return ff;
  }

  public void jogRotateUp() {
    rotate(Constants.Arm.jogSpeedShoulder);
  }

  public void jogRotateDown() {
    rotate(-Constants.Arm.jogSpeedShoulder);
  }

  public void stopShoulder() {
    rotate(0);
  }

  public void extend(double speed) {
    speed = Math.max(Math.min(speed, 1), -1);
    mForearm.set(speed);
  }

  public void extendDistance(double cm) {
    // mForearm.setPosition(cm );
  }

  public void jogUp() {
    extend(Constants.Arm.forearmJogSpeed);
  }

  public void jogDown() {
    extend(-Constants.Arm.forearmJogSpeed);
  }

  public void stopForearm() {
    extend(0);
  }

  public int getShoulderPosition() {
    return mShoulder.getSelectedSensorPosition();
  }

  public int getShoulderVelocity() {
    return mShoulder.getSelectedSensorVelocity();
  }

  public int getForearmPosition() {
    return mForearmEncoder.get();
  }

  @Override
  public void zeroSensors() {
    // TODO Auto-generated method stub
  }

  @Override
  public void checkSubsystem() {
    // TODO Auto-generated method stub
  }

  public void printPIDConstants(){
    /*
_rightMaster.config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
		_rightMaster.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
		_rightMaster.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
    */
   // mShoulde
  }
}
