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
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;


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
  private PowerDistributionPanel powerPanel = new PowerDistributionPanel(0);
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

  private boolean mIsArmJogMode = false;
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // arm extension test controls
    if (mOperatorInterface.getArmExtendManual()) {
      mArm.jogOut();
    } else if (mOperatorInterface.getArmRetractManual()) {
      mArm.jogIn();
    } else {
      if (mOperatorInterface.getArmExtend()) {
        mArm.extendToPosition(Constants.Arm.forearmMaxExtension);
      } else if (mOperatorInterface.getArmRetract()) {
        mArm.extendToPosition(0);
      } else {
        mArm.stopForearm();
      }
    }

    // shoulder test controls
    double target = mArm.getJoystickTarget(mOperatorInterface.getShoulderTargetX(), mOperatorInterface.getShoulderTargetY());

    if (target != mArm.getShoulderTarget()) {
      mIsArmJogMode = false;
    }

    if (mOperatorInterface.getArmJogUp()) {
      mArm.jogRotateUp();
      mIsArmJogMode = true;
    } else if (mOperatorInterface.getArmJogDown()) {
      mArm.jogRotateDown();
      mIsArmJogMode = true;
    } else {
      if (mIsArmJogMode) {
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
    mGrasper.update(powerPanel.getCurrent(Constants.Grasper.pwmChannel));

    // elevator test controls
  }

  private void RobotLoop(){
    // check for change of mode
    checkModeChange();

    if(mCurrentMode == RobotMode.CargoScorer){
      // Objective is to Score Cargo
      // Allow Driver and Operator to control arm and grasper
      ArmTarget target = mOperatorInterface.getArmPos();

      if (mGrasper.getBallsStored() == 2 && target == ArmTarget.INTAKE) {
        target = ArmTarget.SCORE_FRONT;
      }

      if (target != null) {
        mArm.rotateToPosition(target.degrees);
        mArm.extendToPosition(target.distance);
        if (target == ArmTarget.INTAKE) mGrasper.intake();
      } else {
        mArm.rotateToPosition(mArm.getShoulderTarget());
        mArm.extendToPosition(mArm.getForearmTarget());
      }

      if (mOperatorInterface.getArmEject()) mGrasper.eject();
      
      mGrasper.update(powerPanel.getCurrent(Constants.Grasper.pwmChannel));
    } else if(mCurrentMode == RobotMode.Climber) {
      // Objective is to Climb
      // Do not allow manual control of arm and grasper

      // grasper should be stopped, no need in climbing mode
      mGrasper.stop();

      // Allow Operator to stop
      boolean manualStop = false;
      mClimber.update(manualStop);
    }
    DriveLoop();
  }

  // Check for Change of Mode 
  // Controlled by the Operator Controller
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

  private void DriveLoop(){
    double driveThrottle = mOperatorInterface.getDriveThrottle();
    double driveTurn = mOperatorInterface.getDriveTurn();

    boolean wantsAutoSteer = mOperatorInterface.getDriveAutoSteer();
    SmartDashboard.putBoolean("Autosteer", wantsAutoSteer);

    // Get Target within the allowed Threshold
    TargetInfo ti = mVisionManager.getTarget(Constants.TargetType.CAMERA_1_BLUE_CARGO, Constants.Vision.kAllowedSecondsThreshold);
    boolean validTargetInfo = (ti != null);
    if(validTargetInfo){
      SmartDashboard.putBoolean("Valid Target", ti.isValid());
      SmartDashboard.putNumber("Target Angle", ti.getErrorAngle());
    }

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
}
