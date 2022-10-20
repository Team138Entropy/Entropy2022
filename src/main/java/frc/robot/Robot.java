// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TargetType;
import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmTarget;
import frc.robot.subsystems.Climber.ClimberTarget;
import frc.robot.vision.TargetInfo;
import frc.robot.vision.VisionManager;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.TrajectoryGeneratorHelper;
import frc.robot.auto.modes.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {  
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTableEntry ballColorEntry;
  // Controllers Reference
  private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

  // Vision Manager
  private final VisionManager mVisionManager = VisionManager.getInstance();

  // Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  
  // Subsystems
  private final Drive mDrive = Drive.getInstance();

  // Autonomous Execution Thread
  private AutoModeExecutor mAutoModeExecutor = null;

  // Autonomous Modes
  private SendableChooser<AutoModeBase> mAutoModes;

  private static SendableChooser<Integer> mBallColorSelctor;

  private boolean inAutoMode = false;
  private boolean inTeleop = false;
  private Accelerometer accelerometer = new BuiltInAccelerometer();

  //boolean for color of ball selected, red is true and blue is false
  public Boolean selectedColor = false;

  public int ballColor = 0;

  
  

  // Mode
  public enum RobotMode {
    CargoScorer("CargoScorer"),
    Climber("Climber")
    ;
    private final String text;

    RobotMode(final String text) {
      this.text = text; 
    }

    @Override
    public String toString() {
      return text;
    }
  };

  // Robot Starts in CargoScorer Mode
  public RobotMode mCurrentMode = RobotMode.CargoScorer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // populate autonomous list
    populateAutonomousModes();

    // generate generic auto modes to load into JIT
    TrajectoryGeneratorHelper.generateExampleTrajectories();
  }
  
  // Fill Autonomous Modes List
  private void populateAutonomousModes(){
    // Auto Mode
    mAutoModes = new SendableChooser<AutoModeBase>();
    mAutoModes.setDefaultOption("Nothing", new DoNothingMode());
    mAutoModes.addOption("One Ball", new OneBall(true));
    mAutoModes.addOption("TAXI", new OneBall(false));
    mAutoModes.addOption("Two Ball", new TwoOrThreeBall(false));
    mAutoModes.addOption("Three Ball", new TwoOrThreeBall(true));
    mAutoModes.addOption("T3.5_B5", new T35_B5());
    mAutoModes.addOption("TEST", new TEST());
    SmartDashboard.putData(mAutoModes);

    // Ball Selector
    mBallColorSelctor = new SendableChooser<Integer>();
    mBallColorSelctor.setDefaultOption("Blue Ball", 1);
    mBallColorSelctor.addOption("FMS" , 0);
    mBallColorSelctor.addOption("Red Ball", 2);
    SmartDashboard.putData(mBallColorSelctor);
  }


  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    updateRobotSmartDashboard();
  }
  
  //Updates SmartDashboard ;3
  private void updateRobotSmartDashboard() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("accel X", accelerometer.getX());
    SmartDashboard.putNumber("accel Y", accelerometer.getY());
    SmartDashboard.putNumber("accel Z", accelerometer.getZ());
    SmartDashboard.putNumber("drive throttle", mOperatorInterface.getDriveThrottle());
    SmartDashboard.putNumber("drive turn", mOperatorInterface.getDriveTurn());

    mSubsystemManager.updateSmartdashboard();
  }



  /** Called at the Start of Autonomous **/
  @Override
  public void autonomousInit() {
    // Reset AutoMode Executor
    if(mAutoModeExecutor != null) mAutoModeExecutor.reset();

    // Default Robot Mode to CargoScorer
    mCurrentMode = RobotMode.CargoScorer;
    
    // Disable Operator Rumble
    mOperatorInterface.setOperatorRumble(false);

    // zero sensors (if not zero'ed prior on this powerup)
    mSubsystemManager.zeroSensorsIfFresh();

    // Get Selected AutoMode
    mAutoModeExecutor.setAutoMode(mAutoModes.getSelected());

    // Start Autonomous Thread
    // This thread will run until disabled
    mAutoModeExecutor.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Autonomous is run through the AutoModeExecutor
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {        
    mOperatorInterface.setOperatorRumble(false);
        
    // zero sensors (if not zero'ed prior on this powerup)
    mSubsystemManager.zeroSensorsIfFresh();
    
    // Disable Auto Thread (if running)
    if (mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
    }

    // Zero Drive Sensors
    mDrive.zeroSensors();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    DriveLoop(false, false);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    // Reset all auto mode state.
    if (mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
    }

    // create Auto Mode Executor
    mAutoModeExecutor = new AutoModeExecutor();
  }

  int mRumbleTimer = 0;
  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // Activate rumble on op controller every second or so
    if (mRumbleTimer > 2000){ mOperatorInterface.setOperatorRumble(true); }
    if (mRumbleTimer > 2025){ mOperatorInterface.setOperatorRumble(false); mRumbleTimer = 0; }
    mRumbleTimer++;
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
   
  }

  /**
   * DriveLoop
   * precisionSteer - Tunes Down Throttle. Useful for precise movements
   * allowAutoSteer - Enables/Disables AutoSteering
   */
  private void DriveLoop(boolean precisionSteer, boolean allowAutoSteer){
    double driveThrottle = mOperatorInterface.getDriveThrottle()*-1;
    double driveTurn = mOperatorInterface.getDriveTurn();


    // precision steer (slow down throttle if left trigger is held)
   if(precisionSteer) driveThrottle *= .3;

    boolean wantsAutoSteer = false;
    wantsAutoSteer &= allowAutoSteer; //disable if autosteer isn't allowed
    SmartDashboard.putBoolean("Autosteer", wantsAutoSteer);

    // Get Target within the allowed Threshold
    TargetInfo ti = mVisionManager.getSelectedTarget(Constants.Vision.kAllowedSecondsThreshold);
    boolean validTargetInfo = (ti != null);
    double errorAngle = 0;
    boolean validTarget = false;
    if(validTargetInfo){
      // Valid Target Packet
      errorAngle = ti.getErrorAngle();
      validTarget = ti.isValid();
    }
    SmartDashboard.putBoolean("Valid Target", validTarget);
    SmartDashboard.putNumber("Target Angle", errorAngle);
    
    if(wantsAutoSteer && validTargetInfo){
      if(ti.isValid()){ //only allow if valud packet
        // autonomously steering robot towards cargo
        // todo: only allow drive in a certain direction? 
       //mDrive.autoSteer(driveThrottle * .4, ti.getErrorAngle());
       mDrive.driveErrorAngle(driveThrottle * .4, ti.getErrorAngle());
      }else{
        System.out.println("Invalid Packet!");
      }
    }else if(wantsAutoSteer){
      // wants auto steer, but invalid target info
      // TODO: vibrate controller so driver knows
    }else{
      //manual drive
      mDrive.setDrive(driveThrottle, driveTurn, false);
    }
  }

}