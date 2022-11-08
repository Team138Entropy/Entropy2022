package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.Robot;
import frc.robot.util.DriveSignal;
import frc.robot.util.Util;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.geometry.Rotation2d;
import frc.robot.util.geometry.Twist2d;
import frc.robot.util.simulation.DriveSimSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.drivers.CTREUnits;
import frc.robot.util.drivers.EntropyTalonFX;
import frc.robot.util.drivers.MotorConfigUtils;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {
  private static Drive mInstance;

  // Drive Talons
  private EntropyTalonFX mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
  WPI_TalonSRX mLeftMasterSRX, mRightMasterSRX;

  // Potential Drive Modes
  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // autonomous 
    AUTO_STEERING // steering is controlled, but user controls throttle
  }

  // Mode of Drive Control
  private DriveControlState mDriveControlState;

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry mOdometry;

  private static final double kTrackWidth = 0.3; // meters (23.5 inches)
  private final DifferentialDriveKinematics mKinematics = 
    new DifferentialDriveKinematics(kTrackWidth);

  private Pose2d mStoredPose;

  // FeedForwardController for Autonomous Use
  // ks = static gain
  // kv = velocity gain
  double ks = 1;
  double kv = 3;
  private final SimpleMotorFeedforward mFeedForward = new SimpleMotorFeedforward(ks, kv);

  //Drive Values
  //v 12 ft/s
  //a 6 ft/s2
  //
  // 1.8288 
  // .5

  // Autonomous PID Controllers
  private final PIDController mLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController mRightPIDController = new PIDController(1, 0, 0);

  // Drivetrain Simulation 
  DriveSimSystem mDriveSimSystem;

  public static synchronized Drive getInstance() {
    if (mInstance == null) {
      mInstance = new Drive();
    }
    return mInstance;
  }

  // Drive Memory for Drive Smoothing
  private final int previousDriveSignalCount = 1;
  private DriveSignal previousDriveSignals[] = new DriveSignal[previousDriveSignalCount];
  private final double drivetrainTicksPerMeter = 22000.0 * 2.08; //using constants now
  
  private Drive() {
    // Create Talon References
    mLeftSlave = new EntropyTalonFX(Constants.Talons.Drive.leftMaster, Constants.Drive.Encoders.ticksPerMeters, 
      MotorConfigUtils.POSITION_SLOT_IDX, MotorConfigUtils.VELOCITY_SLOT_IDX);
    mLeftMaster = new EntropyTalonFX(Constants.Talons.Drive.leftSlave, Constants.Drive.Encoders.ticksPerMeters, 
      MotorConfigUtils.POSITION_SLOT_IDX, MotorConfigUtils.VELOCITY_SLOT_IDX);
    mRightSlave = new EntropyTalonFX(Constants.Talons.Drive.rightMaster, Constants.Drive.Encoders.ticksPerMeters, 
        MotorConfigUtils.POSITION_SLOT_IDX, MotorConfigUtils.VELOCITY_SLOT_IDX);
    mRightMaster = new EntropyTalonFX(Constants.Talons.Drive.rightSlave, Constants.Drive.Encoders.ticksPerMeters, 
      MotorConfigUtils.POSITION_SLOT_IDX, MotorConfigUtils.VELOCITY_SLOT_IDX);

    // These are so we can get encoders on test bed
    mLeftMasterSRX = new WPI_TalonSRX(Constants.Talons.Drive.leftSlave);
    mRightMasterSRX = new WPI_TalonSRX(Constants.Talons.Drive.rightSlave);


    // Configure Each TalonFX 
    MotorConfigUtils.setDefaultTalonFXConfig(mLeftSlave);
    MotorConfigUtils.setDefaultTalonFXConfig(mLeftMaster);
    MotorConfigUtils.setDefaultTalonFXConfig(mRightSlave);
    MotorConfigUtils.setDefaultTalonFXConfig(mRightMaster);

    // Invert Right Encoder Value
    //mRightMaster.invertEncoder();
    mRightMaster.setInverted(true);
    mRightSlave.setInverted(true);

    // Configure Brake Modes
    mLeftMaster.setNeutralMode(NeutralMode.Brake);
    mLeftSlave.setNeutralMode(NeutralMode.Brake);
    mRightMaster.setNeutralMode(NeutralMode.Brake);
    mRightSlave.setNeutralMode(NeutralMode.Brake);

    // Configure slave Talons to follow masters
    mLeftSlave.follow(mLeftMaster);
    mRightSlave.follow(mRightMaster);
    
    // reset gyro to have position to zero
    m_gyro.reset();

    // Reset Odometrey 
    mOdometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    // Init Drive Memory to Zero
    intializeDriveMemory();

    // Default Robot into Open Loop
    setOpenLoop(DriveSignal.NEUTRAL);

    // Initialize Drive Simulation System
    mDriveSimSystem = new DriveSimSystem();
  }

  private void configTalon(EntropyTalonFX talon) {
    talon.configFactoryDefault();
    talon.configNominalOutputForward(0., 0);
    talon.configNominalOutputReverse(0., 0);
    talon.configPeakOutputForward(1, 0);
    talon.configPeakOutputReverse(-1, 0);
    talon.configOpenloopRamp(0);
    talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    talon.setSensorPhase(true);
    talon.setNeutralMode(NeutralMode.Brake);

    // Configure Talon gains
    double P, I, D;

    P = Constants.Drive.AutoPID.p;
    I = Constants.Drive.AutoPID.i;
    D = Constants.Drive.AutoPID.d;


    talon.config_kP(0, P);
    talon.config_kI(0, I);
    talon.config_kD(0, D);
    talon.config_kF(0, 0);
    talon.configClosedLoopPeriod(0, 10);

    talon.configMotionCruiseVelocity(900);
    talon.configMotionAcceleration(750);

    // Acceleration Limits
    // This should be ZERO!
    talon.configOpenloopRamp(0);
  }

  // Intialize Drive Memory Objects
  // Used to smooth cheesydrive
  private void intializeDriveMemory(){
    previousDriveSignals[0] = new DriveSignal(0, 0);
  }

  // Zeros all Drive related sensors
  public void zeroSensors() {
    zeroEncoders(); //zero encoders
    zeroHeading(); //zero gyro
  }

  // Used for Test
  public void setSimplePercentOutput(DriveSignal signal) {
    mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
    mRightMaster.set(ControlMode.PercentOutput, signal.getRight() * -1);
  }

  // Set Open Loop Control
  public synchronized void setOpenLoop(DriveSignal signal) {
      if (mDriveControlState != DriveControlState.OPEN_LOOP) {
          mDriveControlState = DriveControlState.OPEN_LOOP;
      }

      mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
      mRightMaster.set(ControlMode.PercentOutput, signal.getRight());
  }


  public synchronized void setDrive(double throttle, double wheel, boolean quickTurn) {
    throttle = throttle*-1;
    DriveSignal s = getCheesyBrianDrive(throttle, wheel, quickTurn);
    setOpenLoop(s);    
  }

  public synchronized void setUnrampedDrive(double throttle, double wheel, boolean quickTurn) {
    DriveSignal s = getCheesyDrive(throttle, wheel, quickTurn);
    setOpenLoop(s);    
  }
  // Original Cheesy Drive Equation
  // Depcreated for memory system
  public DriveSignal getCheesyDrive(double throttle, double wheel, boolean quickTurn) {
    wheel = wheel * -1; //invert wheel
  
    //if throttle over a speed
    if (Util.epsilonEquals(throttle, 0.0, 0.05)) {
        throttle = 0.0;
        quickTurn = true;
    }

    if (Util.epsilonEquals(wheel, 0.0, 0.045)) {
        wheel = 0.0;
    }
    //Graph to see on desmos
    //y=\left(x\cdot.9\right)^{2}\ \left\{0<x\ \ \le1\right\}
 
    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = wheel / (denominator * denominator) * Math.abs(throttle);        
    }

    if(quickTurn){
      if(wheel > 0){
        wheel = Math.pow((wheel*.5),2);
      }
      else if(wheel < 0){
        wheel = Math.pow((wheel*.5),2) * -1;
      }
    }

    wheel *= kWheelGain;
    DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
    double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
    return new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor);
  }


  // Cheesy Drive with a memory system
  // This loop will calculate the value for the NEXT loop
  // if this is not the first loop,  we will consider the previous loop
  // returns a drive signal
  public DriveSignal getCheesyBrianDrive(double throttle, double wheel, boolean quickTurn) {
    double DeltaVelocityLimit = 0.02;
    throttle = throttle*0.85;
    wheel = wheel * -1; //invert wheel
    if (Util.epsilonEquals(throttle, 0.0, 0.05)) {
        throttle = 0.0;
        quickTurn = true;

        wheel = wheel * 0.25;
    }
    if (Util.epsilonEquals(wheel, 0.0, 0.045)) {
        wheel = 0.0;
    }
    //Graph to see on desmos
    //y=\left(x\cdot.9\right)^{2}\ \left\{0<x\ \ \le1\right\}

    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= kWheelGain;
    DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
    double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
    DriveSignal calculatedDS = new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor);

    DriveSignal currentCommand = previousDriveSignals[0];
    // verify the difference between the two signals is no more than the velocity limit
    // left
    if(Math.abs(currentCommand.getLeft() - calculatedDS.getLeft()) > DeltaVelocityLimit){
      calculatedDS.setLeft(
        currentCommand.getLeft() + (Math.signum(calculatedDS.getLeft() - currentCommand.getLeft()) * DeltaVelocityLimit)
      );
    }

    // right
    if(Math.abs(currentCommand.getRight() - calculatedDS.getRight()) > DeltaVelocityLimit){
      calculatedDS.setRight(
        currentCommand.getRight() + (Math.signum(calculatedDS.getRight() - currentCommand.getRight()) * DeltaVelocityLimit)
      );
    }
    
    // store this value
    previousDriveSignals[0] = calculatedDS;       
    return currentCommand;
  }

  public synchronized void autoSteer(double throttle, double angle){
    double radians = (0.0174) * angle;
    double heading_error_rad = radians;
    final double kAutosteerKp = 0.1;
    boolean towards_goal = true;
    boolean reverse = false;
    double curvature = (towards_goal ? 1.0 : 0.0) * heading_error_rad * kAutosteerKp;
    double DY = throttle;
    if (throttle == 0) {
      throttle = -.18;
    }
    double dtheta = curvature * throttle * (reverse ? -1.0 : 1.0);
    setOpenLoop(Kinematics.inverseKinematics(new Twist2d(DY, 0.0, dtheta)));
  }

  // periodic update 
  public void periodic() {

  }


  // Test all Sensors in the Subsystem
  public void checkSubsystem() {


  }

  public void updateSmartDashBoard() {
    SmartDashboard.putNumber("encoder_left", getLeftEncoderPosition());
    SmartDashboard.putNumber("encoder_right", getRightEncoderPosition());
    SmartDashboard.putNumber("gyro", getHeading());
  }

  // Zero Encoder of Each Falcon500
  public void zeroEncoders() {
    mLeftMaster.zeroEncoder();
    mRightMaster.zeroEncoder();
    mLeftSlave.zeroEncoder();
    mRightSlave.zeroEncoder();
  }

    /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setAutoSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = mFeedForward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = mFeedForward.calculate(speeds.rightMetersPerSecond);

    // get Left and Right encoder rates
    final double leftEncoderRate = mLeftMaster.getRateMetersPerSecond();
    final double rightEncoderRate = mRightMaster.getRateMetersPerSecond();

    // calculate left and right outputs
    final double leftOutput =
        mLeftPIDController.calculate(leftEncoderRate, speeds.leftMetersPerSecond);
    final double rightOutput =
        mRightPIDController.calculate(rightEncoderRate, speeds.rightMetersPerSecond);

    // calculte left and right voltage to feed to motors
    double leftVoltage = leftOutput + leftFeedforward;
    double rightVoltage = rightOutput + rightFeedforward;

    // normalize voltage out of robot voltage (~12)
    // this command from the WPILib is normalized out of 12 
    // Talons expect [1, -1]
    // calculate out of battery voltage
    leftVoltage = leftVoltage/RobotController.getBatteryVoltage();
    rightVoltage = rightVoltage/RobotController.getBatteryVoltage();

    // Invert Right Voltage (same as Teleop)
    //rightVoltage *= -1;

    // set motor outputs
    mLeftMaster.set(ControlMode.PercentOutput, leftVoltage);
    mRightMaster.set(ControlMode.PercentOutput, rightVoltage);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void automousDrive(double xSpeed, double rot) {
    var wheelSpeeds = mKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setAutoSpeeds(wheelSpeeds);
  }

  public synchronized void setPercentOutputDrive(double left, double right){

    mLeftMaster.set(ControlMode.PercentOutput, left);
    mRightMaster.set(ControlMode.PercentOutput, right);
  } 

  //setUnrampedDrive
  // Drives Gyro at a setpoint
  // This function is used to keep the robot straight
  public synchronized void driveGyroSetpoint(double throttle, double gyroAngleSetpoint){
    final double kP = 0.018;
		double turningValue = (gyroAngleSetpoint - (m_gyro.getAngle()*-1)) * kP;
		//turningValue = Math.copySign(turningValue, throttle);
    setUnrampedDrive(throttle, turningValue*-1, true);
    SmartDashboard.putNumber("turn value", turningValue);
    SmartDashboard.putNumber("turn error", gyroAngleSetpoint - (m_gyro.getAngle()*-1));
    System.out.println(gyroAngleSetpoint - (m_gyro.getAngle()*-1));
  }

  /**
   * driveErrorAngle
   * 
   */
  public synchronized void driveErrorAngle(double throttle, double error){
    final double kP = 0.187;
    final double minOutput = 0;
    final double maxOutput = .6155;
    double turningValue = error * kP;

    // Constrain to min output
    if(turningValue < minOutput && turningValue >= 0){
      turningValue = minOutput;
    }else if(turningValue > -minOutput && turningValue <= 0){
      turningValue = -minOutput;
    }

    // Constrain to max output
    if(turningValue > maxOutput){
      turningValue = maxOutput;
    }else if(turningValue < -maxOutput){
      turningValue = -maxOutput;
    }

    // set into drive with no ramp
    setUnrampedDrive(throttle, turningValue, true);
  }

  /** Return the last Drive Signal */
  public synchronized DriveSignal getLastDriveSignal()
  {
    return previousDriveSignals[0];
  }

  /** Update Drivesim */
  public synchronized void updateDriveSim()
  {
    mDriveSimSystem.updateDrive(getLastDriveSignal());
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    System.out.println("updating odometry");
    System.out.println( m_gyro.getRotation2d());
    System.out.println( mLeftMaster.getDistanceMeters());
    System.out.println( mRightMaster.getDistanceMeters());
    mOdometry.update(
        m_gyro.getRotation2d(), mLeftMaster.getDistanceMeters(), mRightMaster.getDistanceMeters()

    );
    System.out.println("complete odometry update");
  }

  /**
   * Resets the field-relative position to a specific location.
   *
   * @param pose The position to reset to.
   */
  public void resetOdometry(Pose2d pose) {
    mOdometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public synchronized Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  /**
   * Stores the Pose of the Robot
   * Useful for generating a backtracked robot path
   */
  public synchronized void storeCurrentPose(){
    mStoredPose = getPose();
  }

  /**
   * Get Stored Pose of RObot
   * @return
   */
  public synchronized Pose2d getStoredPose(){
    return mStoredPose;
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  // Get the left encoder data in meters
  public double getLeftEncoderPosition() {
    // Use SRX class to get encoder because its srx motors
    return CTREUnits.talonPosistionToMeters(mLeftMasterSRX.getSelectedSensorPosition());
  }

  /**
   * Get the encoder data in meters
   */
  // Get the right encoder data in meters
  public double getRightEncoderPosition() {
    // Use SRX class to get encoder because its srx motors
    return CTREUnits.talonPosistionToMeters(mRightMasterSRX.getSelectedSensorPosition());
  }

  public Gyro getGyro(){
    return m_gyro;
  }

  public DifferentialDriveKinematics getKinematics() {
    return mKinematics;
  }
}