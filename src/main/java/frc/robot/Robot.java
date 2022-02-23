// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmTarget;
import frc.robot.subsystems.Climber.ClimberTarget;
import frc.robot.vision.TargetInfo;
import frc.robot.vision.VisionManager;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.modes.*;
import frc.robot.auto.modes.DoNothingMode;
import frc.robot.auto.modes.TestDriveMode;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.PowerDistribution;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {  

  // Controllers Reference
  private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

  // Vision Manager
  private final VisionManager mVisionManager = VisionManager.getInstance();

  // Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  
  // Subsystems
  private final Drive mDrive = Drive.getInstance();
  private final Arm mArm = Arm.getInstance();
  private final Grasper mGrasper = Grasper.getInstance();
  private final Climber mClimber = Climber.getInstance();

  // Autonomous Execution Thread
  private AutoModeExecutor mAutoModeExecutor = null;

  // Autonomous Modes
  private SendableChooser<AutoModeBase> mAutoModes;

  // Booleans for Test Modes
  private boolean mTest_ArmJogging = true;
  private boolean mTest_ClimberJogging = true;

  private boolean inAutoMode = false;
  private boolean inTeleop = false;
  private Accelerometer accelerometer = new BuiltInAccelerometer();

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
  }
  
  // Fill Autonomous Modes List
  private void populateAutonomousModes(){
    mAutoModes = new SendableChooser<AutoModeBase>();
    mAutoModes.setDefaultOption("Nothing", new DoNothingMode());
    mAutoModes.addOption("One Ball", new OneBall());
    mAutoModes.addOption("T3.5_B5", new T35_B5());
   /* mAutoModes.addOption("Test Drive", new TestDriveMode());
    mAutoModes.addOption("Tarmac1_B2_B3_Tarmac2", new Tarmac1_B2_B3_Tarmac2());
    mAutoModes.addOption("DEMO", new DEMO());
    mAutoModes.addOption("TEST", new TEST());
    mAutoModes.addOption("T2_B3_B2_T1", new T2_B3_B2_T1());
    mAutoModes.addOption("T2_B3_T2", new T2_B3_T2());
    mAutoModes.addOption("T1_B2_T1_B3_T2", new T1_B2_T1_B3_T2());
    mAutoModes.addOption("T2_Terminal", new T2_terminal());
    mAutoModes.addOption("T4_Terminal", new T4_terminal());
    */

    SmartDashboard.putData(mAutoModes);
  }

  private boolean robotTippingCheck(){
    boolean isTipping = false;
    if (Math.abs(accelerometer.getX()) > Constants.RobotDimensions.tippingLimitXaxis) {
      isTipping = true;
    }
    return isTipping;
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
    SmartDashboard.putData("power panel",Constants.Grasper.globelPowerDistribution);
    SmartDashboard.putNumber("accel X", accelerometer.getX());
    SmartDashboard.putNumber("accel Y", accelerometer.getY());
    SmartDashboard.putNumber("accel Z", accelerometer.getZ());
    SmartDashboard.putString("Robot Mode", mCurrentMode.toString());
    SmartDashboard.putBoolean("isTipping", robotTippingCheck());
    SmartDashboard.putNumber("drive throttle", mOperatorInterface.getDriveThrottle());
    SmartDashboard.putNumber("drive turn", mOperatorInterface.getDriveTurn());
    mSubsystemManager.updateSmartdashboard();
  }


  /** Called at the Start of Autonomous **/
  @Override
  public void autonomousInit() {
    // Default Robot Mode to CargoScorer
    mCurrentMode = RobotMode.CargoScorer;
    
    mOperatorInterface.setOperatorRumble(false);

    // zero sensors (if not zero'ed prior on this powerup)
    mSubsystemManager.zeroSensorsIfFresh();

    // set auto mode

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
    // Default Robot Mode to CargoScorer
    mCurrentMode = RobotMode.CargoScorer;
    
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
    RobotLoop();
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
    mOperatorInterface.setOperatorRumble(false);

    // Default to Jogging Modes
    mTest_ArmJogging = true;
    mTest_ClimberJogging = true;
  }

  private boolean mIsShoulderJogging = false;
  private boolean mIsForearmJogging = false;
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Zero Sensors (left joystick press)
    if(mOperatorInterface.getTestZeroPress()){
      System.out.println("Zero Pressed!");
      mSubsystemManager.zeroSensors();
    }
    
    // Climber and Jogging Mode Changes
    // Start and Select of the Operator Controller
    if(mOperatorInterface.getSelectButtonPress()){
      mTest_ArmJogging = !mTest_ArmJogging;
    }
    if(mOperatorInterface.getSwitchModePress()){
      mTest_ClimberJogging = !mTest_ClimberJogging;
    }
    
    // Arm is in Jogging Mode or Position Mode
    if(mTest_ArmJogging){
      // Arm Jogging
      SmartDashboard.putString("Arm Test Mode", "Jogging");
      if (mOperatorInterface.getArmJogUp()) {
        mArm.jogRotateUp();
        mIsShoulderJogging = true;
      } else if (mOperatorInterface.getArmJogDown()) {
        mArm.jogRotateDown();
        mIsShoulderJogging = true;
      } else {
        mArm.jogStop();
      }
    }else{
      // Arm Position
      SmartDashboard.putString("Arm Test Mode", "Position");
      double target = mArm.getRotationTarget();
      target = target + (mOperatorInterface.getArmRotateUp() ? 5 : 0);
      target = target - (mOperatorInterface.getArmRotateDown() ? 5 : 0);
      mArm.rotateToPosition(target);
    }
    

    // Climber is in Jogging Mode or Position Mode
    if(mTest_ClimberJogging){
      // Climb Jogging 
      SmartDashboard.putString("Climber Test Mode", "Jogging");
        
      // climber test controls
      if(mOperatorInterface.getClimberTestExtend()){
        System.out.println("climber extend");
        //extend the climber
        mClimber.TestExtend();
      }else if(mOperatorInterface.getClimberTestRetract()){
        System.out.println("climber retract");
        // retract the climber
        mClimber.TestRetract();
      }else{
        // stop the climber
        mClimber.TestStop();
      }
    }else{
      // Climb Position
      SmartDashboard.putString("Climber Test Mode", "Position");
      
      if (mOperatorInterface.getClimberTest()){
        System.out.println("Climber: Go to " + Climber.ClimberTarget.LOW.ticks);
        mClimber.setPosition(50);
      }
      if (mOperatorInterface.getClimberTest2()){
        System.out.println("Climber: Go to " + Climber.ClimberTarget.ABOVE_BAR.ticks);
        mClimber.setPosition(Climber.ClimberTarget.ABOVE_BAR.ticks);


      }
    }

    // arm extension test controls
    if (mOperatorInterface.getArmExtendManual()) {
      mArm.extend();
      mIsForearmJogging = true;
    } else if (mOperatorInterface.getArmRetractManual()) {
      mArm.retract();
      mIsForearmJogging = true;
    } else {
      if (mOperatorInterface.getArmExtend()) {
        mArm.extend();
        mIsForearmJogging = false;
      } else if (mOperatorInterface.getArmRetract()) {
        mArm.retract();
        mIsForearmJogging = false;
      } else if (mIsForearmJogging) {
        mArm.stopForearm();
        mIsForearmJogging = false;
      }
    }



    // grapser test controls
    if (mOperatorInterface.getArmEject()) {
      mGrasper.eject();
    } else if (mOperatorInterface.getGrasperIntakeManual()) {
      mGrasper.intake();
    } else {
      //mGrasper.stop();
    }
    mGrasper.update(Constants.Grasper.globelPowerDistribution.getCurrent(Constants.Grasper.powerDistributionNumber));

    // Run Drive Code! Allow Precision Steer and Auto Aim
    DriveLoop(mOperatorInterface.getDrivePrecisionSteer(), true);
  }

  private ArmTarget lastTarget = ArmTarget.HOME;

  private void RobotLoop(){
    // check for change of mode
    checkModeChange();

    if(mCurrentMode == RobotMode.CargoScorer){
      // Objective is to Score Cargo
      // Allow Driver and Operator to control arm and grasper

      ArmTarget target = mOperatorInterface.getArmPos();

      if (target == null) {
        target = lastTarget;
      }

      if (mGrasper.getBallsStored() < Constants.Grasper.maxBallsStored && target == ArmTarget.INTAKE) {
        mGrasper.intake();
        System.out.println(Constants.Grasper.globelPowerDistribution.getCurrent(Constants.Grasper.powerDistributionNumber));
      } else if (mGrasper.getBallsStored() == Constants.Grasper.maxBallsStored && target == ArmTarget.INTAKE) {
        target = ArmTarget.SCORE_FRONT;
      }

      lastTarget = target;
      
      mArm.rotateToPosition(target.degrees);    
      if (mOperatorInterface.getArmEjectPress()) mArm.requestPunch(); // first press of trigger, starts punch  
      if (mOperatorInterface.getArmEject()) mGrasper.eject(); // on held

      mArm.updateExtensionLoop();
      
      mGrasper.update(Constants.Grasper.globelPowerDistribution.getCurrent(Constants.Grasper.powerDistributionNumber));

      // Drive with Precision Steer and Auto Steer
      DriveLoop(mOperatorInterface.getDrivePrecisionSteer(), true);
    } else if(mCurrentMode == RobotMode.Climber) {
      // Objective is to Climb
      // Do not allow manual control of arm and grasper

      // grasper should be stopped, no need in climbing mode
      mGrasper.stop();

      // Allow Operator to stop
      // first stage might require manual control
      boolean manualStop = false;

      // Update the Climber, manual stop and climber press
      mClimber.update(manualStop, mOperatorInterface.getOperatorClimbStageApprovePress());

      // Drive with Precision Steer Automatically Enabled, no auto steer
      DriveLoop(true, false);
    }
  }

  /**
   * DriveLoop
   * precisionSteer - Tunes Down Throttle. Useful for precise movements
   * allowAutoSteer - Enables/Disables AutoSteering
   */
  private void DriveLoop(boolean precisionSteer, boolean allowAutoSteer){
    double driveThrottle = mOperatorInterface.getDriveThrottle();
    double driveTurn = mOperatorInterface.getDriveTurn();

    // precision steer (slow down throttle if left trigger is held)
   if(precisionSteer) driveThrottle *= .3;

    boolean wantsAutoSteer = mOperatorInterface.getDriveAutoSteer();
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
        SmartDashboard.putNumber("Vision Error Angle", ti.getErrorAngle());
        mDrive.autoSteer(driveThrottle, -1 * ti.getErrorAngle());
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

  // Check for Change of Mode 
  // Controlled by the Start Button on the Operator Controller
  // Also will stop other functions of robot on change
  private void checkModeChange(){
    // Select Button is used to toggle from CargoScorer to Climber
    if(mOperatorInterface.getSwitchModePress()){
      switch(mCurrentMode){
        case CargoScorer:
          // going from cargo scorer to climber
          mGrasper.stop();

          // reset climber 
          mClimber.reset();

          // update current mode
          mCurrentMode = RobotMode.Climber;
        break;
        case Climber:
          // going from climber to cargo scorer
          mGrasper.stop();

          mCurrentMode = RobotMode.CargoScorer;
        break;
        default:
        break;
      }
    }
  }


}
