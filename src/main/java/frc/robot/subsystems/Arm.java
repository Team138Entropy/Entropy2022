package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

/**
 * Arm subsystem comprised of a rotating shoulder and telescoping forearm.
 */
public class Arm extends Subsystem {

  // Motors
  private TalonSRX mShoulder;
  private TalonSRX mForearm;

  private static Arm mInstance;

  // Other variables
  private boolean isAutomaticallyExtending = false;
  private boolean isManuallyExtending = false;
  private boolean wasManuallyExtending = false;
  private double shoulderTarget = Constants.Arm.shoulderStartPosition;

  // The fixed list of targets that we navigate to when using the joystick control
  private int[] targets = {-50, 0, 60, 90, 120, 210};

  public Arm() {
    mShoulder = new TalonSRX(Constants.Talons.Arm.shoulder);
    mForearm = new TalonSRX(Constants.Talons.Arm.forearm);

    // Sensor is flipped, TODO: Tell mechanical to stop eating crayons and fix it
    mShoulder.setSelectedSensorPosition(-Constants.Arm.shoulderStartPosition);
    mShoulder.setSensorPhase(true);
    
    // Configure Sensor Feedback
    // PID constants
    mShoulder.config_kF(0, 1, 10);
    mShoulder.config_kP(0, 33, 10);
		mShoulder.config_kI(0, .01, 10);
    mShoulder.config_kD(0, 330, 0);
    
    // TODO: Check if we should be using the overloaded method with 3 arguments
    mShoulder.configSelectedFeedbackCoefficient(360d / Constants.Arm.shoulderTicksPerRotation);

    //TODO: Configure forearm
  }

  public static Arm getInstance() {
    if (mInstance == null) {
      mInstance = new Arm();
    }
    return mInstance;
  }

  /**
   * Updates the arm trajectory and movement, should be called every robot loop.
   */
  public void update(double joystickX, double joystickY) {
    rotateShoulderPosition(getJoystickTarget(joystickX, joystickY));
    
    wasManuallyExtending = isManuallyExtending;
    if (!isAutomaticallyExtending && !wasManuallyExtending) stopForearm();
  }

  /**
   * Rotate the shoulder at a percent output.
   * @param output percent power between -1 and 1
   */
  public void rotate(double output) {
    mShoulder.set(ControlMode.PercentOutput, output);
  }

  /**
   * Stop shoulder rotation.
   */
  public void stopShoulder() {
    rotate(0);
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
    degrees = Math.max(degrees, Constants.Arm.shoulderMinPosition);
    degrees = Math.min(degrees, Constants.Arm.shoulderMaxPosition);
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
    rotate(Constants.Arm.shoulderJogSpeed);
  }

  /**
   * Jog the shoulder down at a constant power.
   */
  public void jogRotateDown() {
    rotate(-Constants.Arm.shoulderJogSpeed);
  }

  /**
   * Determines which angle the arm should target based on joystick input.
   * @param joystickX the joystick x input, -1 to 1
   * @param joystickY the joystick y input, -1 to 1
   * @return the angle to target
   */
  private double getJoystickTarget(double joystickX, double joystickY) {
    
    // Convert the joystick x and y positions into a 2D vector. Magnitude = sqrt(x^2+y^2).
    // Angle (in radians) = arctangent(y / x)
    double magnitude = Math.sqrt(Math.pow(joystickY, 2) + Math.pow(joystickX, 2));
    double angle = Math.atan(joystickY / joystickX);
    if (magnitude < .5) return shoulderTarget; // If they aren't moving the joystick, don't change the current target
    
    // Java arctangent method returns a number between -pi/2 to pi/2 (-90 to 90 degrees), so we have to convert to
    // degrees. We have to add 180 degrees if we want the arm to be able to rotate past 90
    angle *= -1 / Constants.Misc.degreeToRadian;
    if (joystickX > 0) angle += 180;

    // The difference between each listed angle, and the actual joystick angle, used to find which listed angle is closest to the joystick
    double difference = 360;
    double target = shoulderTarget;
    for (int i : targets) {
      if (Math.abs(i - (int) angle) < difference) {
        target = i + (i < 90 ? 3 : -3); // Offset the target by 3 degrees upwards to compensate for gravity/slop
        difference = Math.abs(i - (int) angle); // Set the new difference
      }
    }

    return target;
  }

  /**
   * Extend the forearm at a percent output between -.9 and .9.
   * @param power percent power between -.9 and .9
   */
  public void extend(double power) {
    power = Math.max(Math.min(power, .9), -.9);

    mForearm.set(ControlMode.PercentOutput, power);
  }

  public void extendToMax() {
    extend(Constants.Arm.forearmExtendSpeed);
    isAutomaticallyExtending = true;
  }

  public void retractToMin() {
    extend(-Constants.Arm.forearmExtendSpeed);
    isAutomaticallyExtending = true;
  }

  /**
   * Extend the forearm at a fixed output. Will automatically stop once this method stops being called.
   * @param power percent power between -.9 and .9
   */
  public void jogOut() {
    extend(Constants.Arm.forearmJogSpeed);
    isAutomaticallyExtending = false;
    isManuallyExtending = true;
  }

  public void jogIn() {
    extend(-Constants.Arm.forearmJogSpeed);
    isAutomaticallyExtending = false;
    isManuallyExtending = true;
  }
  
  /**
   * Stop the forearm motion.
   */
  public void stopForearm() {
    extend(0);
    isAutomaticallyExtending = false;
    isManuallyExtending = false;
  }

  /**
   * @return shoulder position in degrees
   */
  public int getShoulderPosition() {
    return mShoulder.getSelectedSensorPosition();
  }

  /**
   * @return shoulder velocity in degrees/second
   */
  public int getShoulderVelocity() {
    return mShoulder.getSelectedSensorVelocity();
  }
  
  /**
   * @return shoulder motor percent power output
   */
  public double getShoulderOutput() {
    return mShoulder.getMotorOutputPercent();
  }

  /**
   * @return shoulder target position
   */
  public double getShoulderTarget() {
    return shoulderTarget;
  }

  /**
   * @return forearm motor percent power output
   */
  public double getForearmOutput() {
    return mForearm.getMotorOutputPercent();
  }
  
  /**
   * @return whether the forearm is extended fully
   */
  public boolean isForearmExtended() {
    return mForearm.getSensorCollection().isFwdLimitSwitchClosed();
  }

  /**
   * @return whether the forearm is retracted fully
   */
  public boolean isForearmRetracted() {
    return mForearm.getSensorCollection().isRevLimitSwitchClosed();
  }

  /**
   * @return whether the forearm is in between the extended and retracted positions
   */
  public boolean isForearmInBetween() {
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
