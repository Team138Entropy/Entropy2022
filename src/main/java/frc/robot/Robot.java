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

  private boolean inAutoMode = false;
  private boolean inTeleop = false;
  private PowerDistribution powerPanel = new PowerDistribution(0, PowerDistribution.ModuleType.kCTRE);
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
    mAutoModes.addOption("Test Drive", new TestDriveMode());
    mAutoModes.addOption("Tarmac1_B2_B3_Tarmac2", new Tarmac1_B2_B3_Tarmac2());
    mAutoModes.addOption("One Ball", new OneBall());
    mAutoModes.addOption("DEMO", new DEMO());
    mAutoModes.addOption("TEST", new TEST());
    mAutoModes.addOption("T1_B2_T1", new T1_B2_T1());
    mAutoModes.addOption("T2_B3_B2_T1", new T2_B3_B2_T1());
    mAutoModes.addOption("T2_B3_T2", new T2_B3_T2());
    mAutoModes.addOption("T1_B2_T1_B3_T2", new T1_B2_T1_B3_T2());
    mAutoModes.addOption("T2_Terminal", new T2_terminal());
    mAutoModes.addOption("T4_Terminal", new T4_terminal());
    SmartDashboard.putData(mAutoModes);
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
    
    // Put PowerDistributionBoard stats onto the smart dashboard
    // SmartDashboard.putNumber("PDP-Temp", m_pdp.getTemperature());
    // SmartDashboard.putNumber("PDP-Voltage", m_pdp.getVoltage());
    // for (int i = 0; i < 15; i++){ // This way we can get all the channels info
    //   SmartDashboard.putNumber(("PDP-Current-"+i), m_pdp.getCurrent(i));
    // }
    // SmartDashboard.updateValues();

    SmartDashboard.putData("power panel",powerPanel);
    SmartDashboard.putNumber("accel X", accelerometer.getX());
    SmartDashboard.putNumber("accel Y", accelerometer.getY());
    SmartDashboard.putNumber("accel Z", accelerometer.getZ());
    SmartDashboard.putString("Robot Mode", mCurrentMode.toString());
    mSubsystemManager.updateSmartdashboard();
  }


  /** Called at the Start of Autonomous **/
  @Override
  public void autonomousInit() {
    mOperatorInterface.setRumble(false);

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
    mOperatorInterface.setRumble(false);
        
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
    if (mRumbleTimer > 100){ mOperatorInterface.setRumble(true); }
    if (mRumbleTimer > 200){ mOperatorInterface.setRumble(false); mRumbleTimer = 0; }
    mRumbleTimer++;
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    mOperatorInterface.setRumble(false);

  }

  private boolean mIsShoulderJogging = false;
  private boolean mIsForearmJogging = false;
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
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

    // shoulder test controls
    double target = mArm.getJoystickTarget(mOperatorInterface.getShoulderTargetX(), mOperatorInterface.getShoulderTargetY());

    if (target != mArm.getRotationTarget()) {
      mIsShoulderJogging = false;
    }

    if (mOperatorInterface.getArmJogUp()) {
      mArm.jogRotateUp();
      mIsShoulderJogging = true;
    } else if (mOperatorInterface.getArmJogDown()) {
      mArm.jogRotateDown();
      mIsShoulderJogging = true;
    } else {
      if (mIsShoulderJogging) {
        mArm.rotateDistance(0);
      } else {
        mArm.rotateToPosition(target);
      }
    }

    // grapser test controls
    if (mOperatorInterface.getArmEject()) {
      mGrasper.eject();
    } else if (mOperatorInterface.getGrasperIntakeManual()) {
      mGrasper.intake();
    } else {
      mGrasper.stop();
    }
    mGrasper.update(powerPanel.getCurrent(Constants.Grasper.powerDistributionNumber));

    // elevator test controls
    if(mOperatorInterface.getClimberTestExtend()){
      //extend the climber
      mClimber.TestExtend();
    }else if(mOperatorInterface.getClimberTestRetract()){
      // retract the climber
      mClimber.TestRetract();
    }else{
      // stop the climber
      mClimber.TestStop();
    }
  }

  private void RobotLoop(){
    // check for change of mode
    checkModeChange();

    if(mCurrentMode == RobotMode.CargoScorer){
      // Objective is to Score Cargo
      // Allow Driver and Operator to control arm and grasper

      ArmTarget target = mOperatorInterface.getArmPos();

      // TODO Rework this logic
      if (mGrasper.getBallsStored() < Constants.Grasper.maxBallsStored && target == ArmTarget.INTAKE) {
        mGrasper.intake();
      } else if (mGrasper.getBallsStored() == Constants.Grasper.maxBallsStored && target == ArmTarget.INTAKE) {
        target = ArmTarget.SCORE_FRONT;
      }

      
      mArm.rotateToPosition(target.degrees);
      if (target.isExtended) mArm.extend();
      else mArm.retract();
      
      if (mOperatorInterface.getArmEject()) mGrasper.eject();
      
      mGrasper.update(powerPanel.getCurrent(Constants.Grasper.powerDistributionNumber));
    } else if(mCurrentMode == RobotMode.Climber) {
      // Objective is to Climb
      // Do not allow manual control of arm and grasper

      // grasper should be stopped, no need in climbing mode
      mGrasper.stop();

      // Allow Operator to stop
      // first stage might require manual control
      boolean manualStop = false;
      mClimber.update(manualStop);
    }
    DriveLoop();
  }

  private void DriveLoop(){
    double driveThrottle = mOperatorInterface.getDriveThrottle();
    double driveTurn = mOperatorInterface.getDriveTurn();

    boolean wantsAutoSteer = mOperatorInterface.getDriveAutoSteer();
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
