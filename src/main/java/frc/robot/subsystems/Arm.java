package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import frc.robot.Constants;

/**
 * Arm subsystem comprised of a rotating shoulder and telescoping forearm.
 */
public class Arm extends Subsystem {
  private static Arm mInstance;

  private TalonSRX mShoulder;
  private Talon mForearm;
  private Encoder mForearmEncoder;

  public Arm() {
    mShoulder = new TalonSRX(Constants.Talons.Arm.shoulder);
    mForearm = new Talon(Constants.Arm.forearmChannel);
    mForearmEncoder = new Encoder(0, 1);

    // Sensor is flipped, TODO: Tell mechanical to stop eating crayons and fix it
    //mShoulder.setSensorPhase(true);
    mShoulder.setSelectedSensorPosition(-60);
    System.out.println("Starting Position: " + getShoulderPosition());
    
    // Configure Sensor Feedback
    // PID constants
    mShoulder.config_kP(0, 0, 10);
		mShoulder.config_kI(0, 0, 10);
    mShoulder.config_kD(0, 0, 10);
    
    mShoulder.configSelectedFeedbackCoefficient(360d / Constants.Arm.ticksPerRotationShoulder);

    //mShoulder.ConfigSelectedFeedbackCoefficient(kTurnTravelUnitsPerRotation / kEncoderUnitsPerRotation, 1, 10);
    
    //TODO: Configure forearm here
  }

  /** Joe method - use jog instead*/
  public void testRotate() {
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

  /**
   * Rotate the shoulder at a percent speed.
   * @param speed speed based on percent, -1 to 1
   */
  public void rotate(double speed) {
    speed = Math.max(Math.min(speed, 1), -1);
    mShoulder.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Rotate the shoulder by a certain amount of degrees.
   * @param degrees the amount of degrees to rotate by
   */
  public void rotateShoulderDistance(double degrees) {
    rotateShoulderPosition(degrees + getShoulderPosition());
  }

  /**
   * Rotate the shoulder to a position.
   * @param degrees the position to rotate to
   */
  public void rotateShoulderPosition(double degrees) {
    degrees = Math.max(degrees, Constants.Arm.minEncoderPositionShoulder);
    degrees = Math.min(degrees, Constants.Arm.maxEncoderPositionShoulder);
    double ff = getGravityFeedForward();
    System.out.println(ff);
    System.out.println(degrees);
    System.out.println(getShoulderPosition());
    mShoulder.set(ControlMode.MotionMagic, degrees, DemandType.ArbitraryFeedForward, ff);
  }

  /**
   * Find the gravity feed forward value for the motion magic control mode for the shoulder Talon.
   * @return gravity feed forward value
   */
  private double getGravityFeedForward() {
    double currentRads = getShoulderPosition() * Constants.Misc.degreeToRadian;
    double ff = Constants.Arm.maxGravityFF * Math.cos(currentRads);
    return ff;
  }

  /**
   * Jog the shoulder up at the speed defined in Constants.
   */
  public void jogRotateUp() {
    rotate(Constants.Arm.jogSpeedShoulder);
  }

  /**
   * Jog the shoulder down at the speed defined in Constants.
   */
  public void jogRotateDown() {
    rotate(-Constants.Arm.jogSpeedShoulder);
  }

  /**
   * Stop shoulder rotation.
   */
  public void stopShoulder() {
    rotate(0);
  }

  /**
   * Extend the forearm at a percent speed between -1 and 1.
   * @param speed percent speed between -1 and 1
   */
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
