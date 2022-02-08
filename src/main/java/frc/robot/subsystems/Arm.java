package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Arm subsystem comprised of a rotating shoulder and extending forearm.
 */
public class Arm extends Subsystem {
  private static Arm mInstance;

  // Motors
  private TalonSRX mShoulder;
  private TalonSRX mForearm;

  // Other variables
  private double mShoulderTarget = Constants.Arm.shoulderStartPosition;
  private double mForearmTarget = 0;

  // The fixed list of targets that we navigate to when using the joystick control
  private int[] targets = {-50, 0, 60, 90, 120, 210};

  /** 
   * All useful arm target positions during a match. 
   */
  public static enum ArmTarget {
    SCORE_FRONT(60, Constants.Arm.forearmMaxExtension),
    SCORE_BACK(120, Constants.Arm.forearmMaxExtension),
    INTAKE(-50, 0);

    public double degrees;
    public double distance;

    private ArmTarget(double degrees, double extension) {
      this.degrees = degrees;
      this.distance = extension;
    }
  }

  public Arm() {
    mShoulder = new TalonSRX(Constants.Talons.Arm.shoulder);
    mForearm = new TalonSRX(Constants.Talons.Arm.forearm);

    // Sensor is flipped, TODO Tell mechanical to stop eating crayons and fix it
    mShoulder.setSelectedSensorPosition(-Constants.Arm.shoulderStartPosition);
    mShoulder.setSensorPhase(true);
    
    // Configure Sensor Feedback
    // PID constants
    mShoulder.config_kF(0, 1, 10);
    mShoulder.config_kP(0, 0, 10);
		mShoulder.config_kI(0, 0, 10);
    mShoulder.config_kD(0, 0, 10);
    
    // TODO Check if we should be using the overloaded method with 3 arguments
    mShoulder.configSelectedFeedbackCoefficient(360d / Constants.Arm.shoulderTicksPerRotation);

    mForearm.setSelectedSensorPosition(0);

    // Configure Sensor Feedback
    // PID constants
    mForearm.config_kF(0, 1, 10);
    mForearm.config_kP(0, 0, 10);
		mForearm.config_kI(0, 0, 10);
    mForearm.config_kD(0, 0, 10);

    mForearm.configSelectedFeedbackCoefficient(Constants.Arm.forearmExtensionCM / Constants.Arm.forearmMaxExtension);
  }

  public static Arm getInstance() {
    if (mInstance == null) {
      mInstance = new Arm();
    }
    return mInstance;
  }

  /**
   * Rotate the shoulder at a percent speed.
   * @param speed percent speed between -1 and 1
   */
  public void rotate(double speed) {
    mShoulder.set(ControlMode.PercentOutput, speed);
  }
  
  /**
   * Stop shoulder rotation.
   */
  public void stopShoulder() {
    rotate(0);
    mShoulderTarget = getRotation();
  }

  /**
   * Rotate the shoulder to a position.
   * @param degrees the position to rotate to, in degrees
   */
  public void rotateToPosition(double degrees) {
    degrees = Math.max(degrees, Constants.Arm.shoulderStartPosition);
    degrees = Math.min(degrees, Constants.Arm.shoulderMaxRotation);
    double ff = getGravityFeedForward();
    mShoulder.set(ControlMode.MotionMagic, degrees, DemandType.ArbitraryFeedForward, ff);
  }

  /**
   * Rotate the shoulder by a certain number of degrees.
   * @param degrees the number of degrees to rotate by
   */
  public void rotateDistance(double degrees) {
    rotateToPosition(degrees + getRotation());
  }

  /**
   * Find the gravity feed forward value for the motion magic control mode for the shoulder Talon.
   * @return gravity feed forward value
   */
  public double getGravityFeedForward() {
    double currentRads = getRotation() * Constants.Misc.degreeToRadian;
    double ff = Constants.Arm.shoulderMaxGravityFF * Math.cos(currentRads);
    return ff;
  }

  /**
   * Jog the shoulder up at a constant speed.
   */
  public void jogRotateUp() {
    rotate(Constants.Arm.shoulderJogSpeed);
  }

  /**
   * Jog the shoulder down at a constant speed.
   */
  public void jogRotateDown() {
    rotate(-Constants.Arm.shoulderJogSpeed);
  }

  /**
   * Calculate the target angle for the shoulder based on joystick input.
   * @param joystickX the joystick x input, -1 to 1
   * @param joystickY the joystick y input, -1 to 1
   * @return the target angle
   */
  public double getJoystickTarget(double joystickX, double joystickY) {
    
    // Convert the joystick x and y positions into a 2D vector. Magnitude = sqrt(x^2+y^2).
    // Angle (in radians) = arctangent(y / x)
    double magnitude = Math.sqrt(Math.pow(joystickY, 2) + Math.pow(joystickX, 2));
    double angle = Math.atan(joystickY / joystickX);
    if (magnitude < .5) return mShoulderTarget; // If they aren't moving the joystick, don't change the current target
    
    // Java arctangent method returns a number between -pi/2 to pi/2 (-90 to 90 degrees), so we have to convert to
    // degrees. We have to add 180 degrees if we want the arm to be able to rotate past 90
    angle *= -1 / Constants.Misc.degreeToRadian;
    if (joystickX > 0) angle += 180;

    // The difference between each listed angle, and the actual joystick angle, used to find which listed angle is closest to the joystick
    double difference = 360;
    double target = mShoulderTarget;
    for (int i : targets) {
      if (Math.abs(i - (int) angle) < difference) {
        target = i + (i < 90 ? 3 : -3); // Offset the target by 3 degrees upwards to compensate for gravity/slop
        difference = Math.abs(i - (int) angle); // Set the new difference
      }
    }

    return target;
  }

  /**
   * Extend the forearm at a percent speed.
   * @param speed percent speed between -.9 and .9
   */
  public void extend(double speed) {
    speed = Math.max(Math.min(speed, .9), -.9);

    mForearm.set(ControlMode.PercentOutput, speed);
  }
  
  /**
   * Stop the forearm.
   */
  public void stopForearm() {
    extend(0);
    mForearmTarget = getExtension();
  }

  /**
   * Extend the forearm to a position.
   * @param position the position to extend to, in centimeters
   */
  public void extendToPosition(double position) {
    position = Math.max(position, 0);
    position = Math.min(position, Constants.Arm.forearmMaxExtension);
    
    mForearm.set(ControlMode.MotionMagic, position);
    mForearmTarget = position;
  }

  /**
   * Extend the forearm by a certain distance.
   * @param distance the distance to extend by, in centimeters
   */
  public void extendDistance(double distance) {
    extendToPosition(distance + getExtension());
  }

  /**
   * Extend the forearm at a fixed speed.
   */
  public void jogExtend() {
    extend(Constants.Arm.forearmJogSpeed);
  }

  /**
   * Retract the forearm at a fixed speed.
   */
  public void jogRetract() {
    extend(-Constants.Arm.forearmJogSpeed);
  }

  /**
   * @return shoulder position in degrees
   */
  public double getRotation() {
    return mShoulder.getSelectedSensorPosition();
  }

  /**
   * @return shoulder velocity in degrees/second
   */
  public double getShoulderVelocity() {
    return mShoulder.getSelectedSensorVelocity() * 10;
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
    return mShoulderTarget;
  }

  /**
   * @return forearm extension in centimeters
   */
  public double getExtension() {
    return mForearm.getSelectedSensorPosition();
  }

  /**
   * @return forearm velocity in cm/s
   */
  public double getForearmVelocity() {
    return mForearm.getSelectedSensorVelocity() * 10;
  }

  /**
   * @return forearm motor percent power output
   */
  public double getForearmOutput() {
    return mForearm.getMotorOutputPercent();
  }

  /**
   * @return forearm target position
   */
  public double getForearmTarget() {
    return mForearmTarget;
  }
  
  /**
   * @return whether the forearm is fully extended
   */
  public boolean isForearmExtended() {
    return mForearm.getSensorCollection().isFwdLimitSwitchClosed();
  }

  /**
   * @return whether the forearm is fully retracted
   */
  public boolean isForearmRetracted() {
    return mForearm.getSensorCollection().isRevLimitSwitchClosed();
  }

  @Override
  public void zeroSensors() {
    // TODO Implement this
  }

  @Override
  public void checkSubsystem() {
    // TODO Implement this
  }

  @Override
  public void updateSmartDashBoard(){
    SmartDashboard.putNumber("shoulderTarget", getShoulderTarget());
    SmartDashboard.putNumber("shoulderPosition", getRotation());
    SmartDashboard.putNumber("shoulderVelocity", getShoulderVelocity());
    SmartDashboard.putNumber("shoulderOutput", getShoulderOutput());
    SmartDashboard.putNumber("forearmTarget", getForearmTarget());
    SmartDashboard.putNumber("forearmPosition", getExtension());
    SmartDashboard.putNumber("forearmVelocity", getForearmVelocity());
    SmartDashboard.putNumber("forearmOutput", getForearmOutput());
  }
}
