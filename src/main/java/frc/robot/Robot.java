// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.*;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.modes.*;
import frc.robot.auto.modes.DoNothingMode;
import frc.robot.auto.modes.TestDriveMode;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {  

  // Controller Reference
  private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

  // Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  
  // Subsystems
  private final Drive mDrive = Drive.getInstance();

  // Autonomous Execution Thread
  private AutoModeExecutor mAutoModeExecutor = null;

  // Autonomous Modes
  private SendableChooser<AutoModeBase> mAutoModes;

  //Power dist. panel
  //private final PowerDistributionPanel m_pdp = new PowerDistributionPanel(25);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SendableChooser<AutoModeBase> mAutoModes = new SendableChooser<>();
    // populate autonomous list
    populateAutonomousModes();
  }
  
  // Fill Autonomous Modes List
  private void populateAutonomousModes(){
    mAutoModes = new SendableChooser<AutoModeBase>();
    mAutoModes.setDefaultOption("Nothing", new DoNothingMode());
    mAutoModes.addOption("Test Drive", new TestDriveMode());
    mAutoModes.addOption("Tarmac1_B2_B3_Tarmac2", new Tarmac1_B2_B3_Tarmac2());
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
    SmartDashboard.putNumber("encoder_left", mDrive.getLeftEncoderPosition());
    SmartDashboard.putNumber("encoder_right", mDrive.getRightEncoderPosition());
    // Put PowerDistributionBoard stats onto the smart dashboard
    // SmartDashboard.putNumber("PDP-Temp", m_pdp.getTemperature());
    // SmartDashboard.putNumber("PDP-Voltage", m_pdp.getVoltage());
    // for (int i = 0; i < 15; i++){ // This way we can get all the channels info
    //   SmartDashboard.putNumber(("PDP-Current-"+i), m_pdp.getCurrent(i));
    // }
    // SmartDashboard.updateValues();
  }


  /** Called at the Start of Autonomous **/
  @Override
  public void autonomousInit() {
    mOperatorInterface.setRumble(false);

    // set auto mode

    // Get Selected AutoMode
    // AutoModeBase selectedMode = mAutoModes.getSelected();
    // if(selectedMode == null){
    //   System.out.println("Selected Auto Mode is Null");
    // }
    //TestDriveMode selectedMode = new TestDriveMode();
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

    teleopRobotLoop();
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

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  private void teleopRobotLoop(){
    teleopDriveLoop();
  }

  private void teleopDriveLoop(){
    double driveThrottle = mOperatorInterface.getDriveThrottle();
    double driveTurn = mOperatorInterface.getDriveTurn();

    //manual drive
    mDrive.setDrive(driveThrottle, driveTurn, false);
  }


}
