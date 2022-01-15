package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

/**
 * Arm subsystem comprised of a rotating shoulder and telescoping forearm.
 */
public class Arm extends Subsystem {
  private static Arm mInstance;

  private TalonSRX mShoulder;
  private TalonSRX mForearm;

  public Arm() {
    mShoulder = new TalonSRX(Constants.Talons.Arm.shoulder);
    mForearm = new TalonSRX(Constants.Talons.Arm.forearm);

    // Sensor is flipped, TODO: Tell mechanical to stop eating crayons and fix it
    mShoulder.setSelectedSensorPosition(60);
    mShoulder.setSensorPhase(true);


    System.out.println("Starting Position: " + getShoulderPosition());
    
    // Configure Sensor Feedback
    // PID constants
    mShoulder.config_kF(0, 1, 10);
    mShoulder.config_kP(0, 33, 10);
		mShoulder.config_kI(0, .01, 10);
    mShoulder.config_kD(0, 330, 0);

    // Testing
    // mShoulder.configClosedLoopPeriod(slotIdx, loopTimeMs)
    
    mShoulder.configSelectedFeedbackCoefficient(360d / Constants.Arm.ticksPerRotationShoulder);

    //mShoulder.ConfigSelectedFeedbackCoefficient(kTurnTravelUnitsPerRotation / kEncoderUnitsPerRotation, 1, 10);
    
    //TODO: Configure forearm here
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
    degrees = Math.max(degrees, Constants.Arm.minPositionShoulder);
    degrees = Math.min(degrees, Constants.Arm.maxPositionShoulder);
    double ff = getGravityFeedForward();
    mShoulder.set(ControlMode.MotionMagic, degrees, DemandType.ArbitraryFeedForward, ff);
  }

  /**
   * Find the gravity feed forward value for the motion magic control mode for the shoulder Talon.
   * @return gravity feed forward value
   */
  public double getGravityFeedForward() {
    double currentRads = getShoulderPosition() * Constants.Misc.degreeToRadian;
    double ff = Constants.Arm.maxGravityFF * Math.cos(currentRads);
    return ff;
  }

  /**
   * Jog the shoulder up at a constant power.
   */
  public void jogRotateUp() {
    rotate(Constants.Arm.jogSpeedShoulder);
  }

  /**
   * Jog the shoulder down at a constant power.
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
   * Extend the forearm at a percent power between -.5 and .5.
   * @param speed percent power between -.5 and .5
   */
  public void extend(double speed) {
    speed = Math.max(Math.min(speed, .9), -.9);
    System.out.println("Forearm::extend! " + speed);

    mForearm.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Jog the forearm out at a constant power.
   */
  public void jogOut() {
    extend(Constants.Arm.forearmJogSpeed);
  }

    /**
   * Jog the forearm in at a constant power.
   */
  public void jogIn() {
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
  
  public double getShoulderOutput() {
    return mShoulder.getMotorOutputPercent();
  }

  public boolean isForearmExtended() {
    return mForearm.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean isForearmMoving() {
    return !(mForearm.getSensorCollection().isFwdLimitSwitchClosed() || mForearm.getSensorCollection().isFwdLimitSwitchClosed());
  }

  @Override
  public void zeroSensors() {
    // TODO Auto-generated method stub
  }

  @Override
  public void checkSubsystem() {
    // TODO Auto-generated method stub
  }
}
