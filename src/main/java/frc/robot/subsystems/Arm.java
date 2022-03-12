package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Arm subsystem comprised of a rotating shoulder and extending forearm.
 */
public class Arm extends Subsystem {
  private static Arm mInstance;

  // Some constants
  private final double kShoulderJogSpeed = .35;
  private final double kShoulderMaxGravityFF = .2;
  private final double kForearmExtendSpeed = 1;

  // Motors
  private TalonSRX mShoulder;
  private TalonSRX mForearm;

  // Other variables
  private double mShoulderTarget = Constants.Arm.shoulderStartPosition;

  // The fixed list of targets that we navigate to when using the joystick control
  private int[] mTargets;

  /** 
   * All useful arm target positions during a match. 
   */
  public static enum ArmTarget {
    SCORE_FRONT(110, false),
    SCORE_BACK(65, false),
    INTAKE(-30, false),
    HOME(90, false),
    FLAT_FRONT(180, false),
    FLAT_BACK(0, false),
    CLIMB_START(155, false);

    public double degrees;
    public boolean isExtended;

    private ArmTarget(double degrees, boolean isExtended) {
      this.degrees = degrees;
      this.isExtended = isExtended;
    }
  }

  public static enum ArmExtensionTarget {
    FULLY_RETRACTED(0),
    MIDWAY(92000),
    ABOVE_HIGH_BAR(80000),
    UNDER_HIGH_BAR(40000),
    SLIGHT_RETRACTION(138000),
    FULLY_EXTENDED(184453);

    public double ticks;

    private ArmExtensionTarget(double tick){
      this.ticks = tick;
    }
  } 


  public Arm() {
    mShoulder = new TalonSRX(Constants.Talons.Arm.shoulder);
    mShoulder.setNeutralMode(NeutralMode.Brake);
    mForearm = new TalonSRX(Constants.Talons.Arm.forearm);
    mForearm.setNeutralMode(NeutralMode.Brake);
    mForearm.setInverted(true);
    mTargets = new int[] {-50, 0, 60, 90, 120, 210};

    mShoulder.setSensorPhase(true);
    mShoulder.setInverted(true);
    
    // PID constants
    // Old constants are 1, 33, .01, 330
   mShoulder.config_kF(0, 1, 10);
    mShoulder.config_kP(0, 35, 10);
		mShoulder.config_kI(0, 0, 10);
    mShoulder.config_kD(0, 0, 10);
    
    mShoulder.configSelectedFeedbackCoefficient(360d / Constants.Arm.shoulderTicksPerRotation);
    mShoulder.configMotionAcceleration(10);
    mShoulder.configMotionCruiseVelocity(25, 10); // originally accel 5 veloc 10

    mForearm.setSensorPhase(true);
    
    // Forearm configuration
    
    mForearm.config_kF(0, 1, 10);
    mForearm.config_kP(0, .9, 10);
		mForearm.config_kI(0, 0, 10);
    mForearm.config_kD(0, 0, 10);
    mForearm.configMotionAcceleration(20000);
    mForearm.configMotionCruiseVelocity(7200, 10); 
  }

  public static Arm getInstance() {
    if (mInstance == null) {
      mInstance = new Arm();
    }
    return mInstance;
  }

  public synchronized void configArmVelocityAndAcceleration(double velocity, double acceleration){
    mShoulder.configMotionAcceleration(acceleration);
    mShoulder.configMotionCruiseVelocity(velocity, 10); // originally accel 5 veloc 10
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
    degrees = Math.max(degrees, Constants.Arm.shoulderMinRotation);
    degrees = Math.min(degrees, Constants.Arm.shoulderMaxRotation);
    double ff = getGravityFeedForward();
    mShoulderTarget = degrees;
    mShoulder.set(ControlMode.MotionMagic, degrees, DemandType.ArbitraryFeedForward, ff);
  }

  /**
   * Rotate the shoulder by a certain number of degrees.
   * @param degrees the number of degrees to rotate by
   */
  public void rotateDistance(double degrees) {
    mShoulderTarget += degrees;
    rotateToPosition(degrees + getRotation());
  }

  /**
   * Find the gravity feed forward value for the motion magic control mode for the shoulder Talon.
   * @return gravity feed forward value
   */
  public double getGravityFeedForward() {
    double currentRads = getRotation() * Constants.Misc.degreeToRadian;
    double ff = kShoulderMaxGravityFF * Math.cos(currentRads);
    return ff;
  }

  /**
   * Jog the shoulder up at a constant speed.
   */
  public void jogRotateUp() {
    rotate(kShoulderJogSpeed);
    
  }

  /**
   * Jog the shoulder down at a constant speed.
   */
  public void jogRotateDown() {
    rotate(-kShoulderJogSpeed);
    
  }

  public void jogStop(){
    rotate(0);
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
    for (int i : mTargets) {
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

  public void extendToPosition(double ticks) {
    mForearm.set(ControlMode.MotionMagic, ticks, DemandType.ArbitraryFeedForward, .3);
  }
  
  /**
   * Stop the forearm.
   */
  public void stopForearm() {
    System.out.println("STOP!");
    extend(0);
  }

  /**
   * Extend the forearm at a fixed speed.
   */
  public void extend() {
    System.out.println("EXTEND!");
    extend(kForearmExtendSpeed);
  }

  /**
   * Retract the forearm at a fixed speed.
   */
  public void retract() {
    System.out.println("RETRACT!");
    extend(-kForearmExtendSpeed);
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
  public double getRotationVelocity() {
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
  public double getRotationTarget() {
    return mShoulderTarget;
  }

  /**
   * @return forearm motor percent power output
   */
  public double getForearmOutput() {
    return mForearm.getMotorOutputPercent();
  }
  
  /**
   * @return whether the forearm is fully extended
   */
  public boolean isExtended() {
    return mForearm.getSensorCollection().isFwdLimitSwitchClosed();
  }

  /**
   * @return whether the forearm is fully retracted
   */
  public boolean isRetracted() {
    return mForearm.getSensorCollection().isRevLimitSwitchClosed();
  }

  // Returns if the Shoulder Position is correct given a threshold
  public boolean isAtPosition(double angle){
    double foundAngle = getRotation();
    return (foundAngle <= angle + 4) && (foundAngle >= angle - 4);
  }

  public boolean isAtExtension(double extPos){
    double extensionPosition = getExtensionPosition();
    return (extensionPosition <= extPos + 300) && (extensionPosition >= extPos - 300);
  }

  
  public double getExtensionPosition(){
    return mForearm.getSelectedSensorPosition();
}

  @Override
  public void zeroSensors() {
    // Sensor is flipped
    mShoulder.setSelectedSensorPosition(Constants.Arm.shoulderStartPosition);
    mForearm.setSelectedSensorPosition(0);
    
    /*
    if (getRotation() < 0) {
      mShoulder.setSelectedSensorPosition(Constants.Arm.shoulderStartPosition);
      mShoulder.setSensorPhase(fa);
    }
    */
    
   }

  @Override
  public void checkSubsystem() {
    // TODO Implement this
  }

  @Override
  public void updateSmartDashBoard(){
    SmartDashboard.putNumber("shoulderTarget", getRotationTarget());
    SmartDashboard.putNumber("shoulderPosition", getRotation());
    SmartDashboard.putNumber("shoulderVelocity", getRotationVelocity());
    SmartDashboard.putNumber("shoulderOutput", getShoulderOutput());
    SmartDashboard.putNumber("shoulder talon output", mShoulder.getMotorOutputPercent());
    SmartDashboard.putNumber("Extension Output", getForearmOutput());
    SmartDashboard.putNumber("Extension Current", mForearm.getSupplyCurrent());
    SmartDashboard.putNumber("Extension Position", getExtensionPosition());
    SmartDashboard.putNumber("Extension Velocity", mForearm.getSelectedSensorVelocity());

  }
}
