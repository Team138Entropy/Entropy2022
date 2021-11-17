package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.Robot;
import frc.robot.util.DriveSignal;
import frc.robot.util.Util;
import frc.robot.util.geometry.Rotation2d;
import frc.robot.util.geometry.Twist2d;

public class Drive extends Subsystem {
  private static Drive mInstance;

  // Drive Talons
  private TalonFX mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;

  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // velocity PID control
  }

  private DriveControlState mDriveControlState;

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  
  private PeriodicDriveData mPeriodicDriveData = new PeriodicDriveData();
  public static class PeriodicDriveData {
    // INPUTS
    public double timestamp;
    public double left_voltage;
    public double right_voltage;
    public int left_position_ticks;
    public int right_position_ticks;
    public double left_distance;
    public double right_distance;
    public int left_velocity_ticks_per_100ms;
    public int right_velocity_ticks_per_100ms;
    public boolean climbingSpeed = false;

    // OUTPUTS
    public double left_demand;
    public double right_demand;
    public double left_feedforward;
    public double right_feedforward;
    public double left_old = 0;
    public double right_old = 0;
    public boolean isQuickturning = false;
  }

  public static synchronized Drive getInstance() {
    if (mInstance == null) {
      mInstance = new Drive();
    }
    return mInstance;
  }

  public int feetToTicks(double feet) {
    double ticks;
    ticks = Constants.Drive.Encoders.compTicksPerFoot;
    
    long roundedVal = Math.round(feet * ticks);
    if (roundedVal > Integer.MAX_VALUE) {
     
    }

    return (int) roundedVal;
  }

  // Drive Memory for Drive Smoothing
  private final int previousDriveSignalCount = 1;
  private DriveSignal previousDriveSignals[] = new DriveSignal[previousDriveSignalCount]; 

  private Drive() {
    mLeftSlave = new TalonFX(Constants.Talons.Drive.leftMaster);
    mLeftMaster = new TalonFX(Constants.Talons.Drive.leftSlave);
    mRightSlave = new TalonFX(Constants.Talons.Drive.rightMaster);
    mRightMaster = new TalonFX(Constants.Talons.Drive.rightSlave);

    configTalon(mLeftMaster);
    mLeftSlave.setNeutralMode(NeutralMode.Brake);
    mLeftSlave.setSensorPhase(false);

    configTalon(mRightMaster);
    mRightSlave.setNeutralMode(NeutralMode.Brake);

    // Configure slave Talons to follow masters
    mLeftSlave.follow(mLeftMaster);
    mRightSlave.follow(mRightMaster);

    setOpenLoop(DriveSignal.NEUTRAL);

    // reset gyro to have psotion zero
    m_gyro.reset();

    // Reset Odometrey 
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    // Encoder Setup
    mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
		mLeftMaster.getSensorCollection().setIntegratedSensorPosition(0, 0);
    mRightMaster.getSensorCollection().setIntegratedSensorPosition(0, 0);
    
    // Init Drive Memory to Zero
    intializeDriveMemory();
  }

  private void configTalon(TalonFX talon) {
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

    // Accelerate Limits
    double secondsToFull = .8;
    talon.configOpenloopRamp(0);
  }

  // Intialize Drive Memory Objects
  private void intializeDriveMemory(){
    previousDriveSignals[0] = new DriveSignal(0, 0);
  }

  public void resetCruiseAndAccel() {

  }

  public void setCruiseAndAcceleration(int cruise, int accel) {
    mLeftMaster.configMotionCruiseVelocity(cruise);
    mRightMaster.configMotionCruiseVelocity(cruise);

    mLeftMaster.configMotionAcceleration(accel);
    mRightMaster.configMotionAcceleration(accel);
  }

  public void configP(double p) {
    mLeftMaster.config_kP(0, p);
    mRightMaster.config_kP(0, p);
  }

  public void configI(double i) {
    mLeftMaster.config_kI(0, i);
    mRightMaster.config_kI(0, i);
  }

  public void configD(double d) {
    mLeftMaster.config_kD(0, d);
    mRightMaster.config_kD(0, d);
  }

  public void resetPID() {
    double P, I, D;

    P = Constants.Drive.AutoPID.p;
    I = Constants.Drive.AutoPID.i;
    D = Constants.Drive.AutoPID.d;

    configP(P);
    configI(I);
    configD(D);
  }

  public void zeroSensors() {
    zeroEncoders();
  }

  public void setMotionMagicTarget(int left, int right) {
    mLeftMaster.set(ControlMode.MotionMagic, left);
    mRightMaster.set(ControlMode.MotionMagic, -right);
  }

  public void setSimplePIDTarget(int left, int right) {
    mLeftMaster.set(ControlMode.Position, left);
    mRightMaster.set(ControlMode.Position, -right);
  }

  /** Configure talons for open loop control */

  // Used for arcade turning during auto
  public void setSimplePercentOutput(DriveSignal signal) {
    mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
    mRightMaster.set(ControlMode.PercentOutput, signal.getRight() * -1);
  }

  // Set Open Loop Control
  public synchronized void setOpenLoop(DriveSignal signal) {
      if (mDriveControlState != DriveControlState.OPEN_LOOP) {
          //setBrakeMode(true);

          mDriveControlState = DriveControlState.OPEN_LOOP;
      }

      mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
      mRightMaster.set(ControlMode.PercentOutput, signal.getRight() * -1);
  }


  public synchronized void setDrive(double throttle, double wheel, boolean quickTurn) {
    DriveSignal s = getCheesyBrianDrive(throttle, wheel, quickTurn);
    setOpenLoop(s);
    System.out.println("Gyro: " + m_gyro.getRotation2d().getDegrees());
    System.out.println("Encoder Left: " + mLeftMaster.getSelectedSensorPosition());
    System.out.println("Encoder Right: " + mRightMaster.getSelectedSensorPosition());
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
        wheel = wheel * .80;
        /*
        if(throttle < 0){
          wheel = wheel * -1;
        }
        */
        
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
    double DeltaVelocityLimit = 0.015;
    wheel = wheel * -1; //invert wheel
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

  // periodic update 
  public void periodic() {
    // Update the odometry in the periodic block
    //m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

  }


  // Test all Sensors in the Subsystem
  public void checkSubsystem() {


  }

 
  public void zeroEncoders() {

  }

}
