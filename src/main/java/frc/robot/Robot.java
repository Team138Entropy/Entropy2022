// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.*;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.DoNothingMode;
import frc.robot.auto.modes.TestDriveMode;
import frc.robot.auto.modes.ControllerModeBase;
import frc.robot.auto.modes.JoysticksMode;
import frc.robot.auto.modes.XboxControllermode;
import frc.robot.Constants.Controllers.Operator;
import frc.robot.Constants;
import frc.robot.OI.XboxController.Button;

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

  // Driver Controller Modes
  private SendableChooser<Integer> mDriverControllerModes;
  
  // Operator Controller Modes
  private SendableChooser<Integer> mOperatorControllerModes;

  private boolean inAutoMode = false;
  private boolean inTeleop = false;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SendableChooser<AutoModeBase> mAutoModes = new SendableChooser<>();
    SendableChooser<Integer> mDriverControllerModes = new SendableChooser<>();
    SendableChooser<Integer> mOperatorControllerModes = new SendableChooser<>();
    // populate autonomous list
    populateAutonomousModes();
  }
  
  // Fill Autonomous Modes List
  private void populateAutonomousModes(){
    mAutoModes = new SendableChooser<AutoModeBase>();
    mAutoModes.setDefaultOption("Nothing", new DoNothingMode());
    mAutoModes.addOption("Test Drive", new TestDriveMode());
    SmartDashboard.putData(mAutoModes);
    mDriverControllerModes = new SendableChooser<Integer>();
    mDriverControllerModes.setDefaultOption("Xbox Controller(Driver)", 0);
    mDriverControllerModes.addOption("Joysticks(Driver)", 1);
    mDriverControllerModes.addOption("Wheel(Driver)", 2);
    mDriverControllerModes.addOption("Thing(Driver)", 3);
    SmartDashboard.putData(mDriverControllerModes);
    mOperatorControllerModes = new SendableChooser<Integer>();
    mOperatorControllerModes.setDefaultOption("Xbox Controller(Operator)", 0);
    mOperatorControllerModes.addOption("Joysticks(Operator)", 1);
    mOperatorControllerModes.addOption("Wheel(Operator)", 2);
    mOperatorControllerModes.addOption("Thing(Operator)", 3);
    SmartDashboard.putData(mOperatorControllerModes);
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
    SmartDashboard.putBoolean("In Auto", inAutoMode);
    SmartDashboard.putBoolean("In Teleop", inTeleop);
    SmartDashboard.putNumber("Speed", mOperatorInterface.getDriveThrottle());
    SmartDashboard.putNumber("voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Left Encoder", mDrive.getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder", mDrive.getRightEncoderPosition());
  }

  /** Called at the Start of Autonomous **/
  @Override
  public void autonomousInit() {
    inAutoMode = true;
    inTeleop = false;
    // Get Selected AutoMode
    AutoModeBase selectedMode = mAutoModes.getSelected();
    if(selectedMode == null){
      System.out.println("Selected Auto Mode is Null");
    }
    //TestDriveMode selectedMode = new TestDriveMode();
    mAutoModeExecutor.setAutoMode(selectedMode);

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
    inTeleop = true;
    inAutoMode = false;
    // Disable Auto Thread (if running)
    if (mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
    }
    Integer selectedDriveController = mDriverControllerModes.getSelected();
    Integer selectedOperatorController = mOperatorControllerModes.getSelected();
    if(selectedDriveController == 0){
      mOperatorInterface.setDriveControllerInUse(0);
    }
    if(selectedDriveController == 1){
      mOperatorInterface.setDriveControllerInUse(1);
    }
    if(selectedDriveController == 2){
      mOperatorInterface.setDriveControllerInUse(2);
    }
    if(selectedDriveController == 3){
      mOperatorInterface.setDriveControllerInUse(3);
    }
    if(selectedOperatorController == 0){
      mOperatorInterface.setOperatorControllerInUse(0);
    }
    if(selectedOperatorController == 1){
      mOperatorInterface.setOperatorControllerInUse(1);
    }
    if(selectedOperatorController == 2){
      mOperatorInterface.setOperatorControllerInUse(2);
    }
    if(selectedOperatorController == 3){
      mOperatorInterface.setOperatorControllerInUse(3);
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

    inAutoMode = false;
    inTeleop = false;
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // Set Mode for the Auto Mode to use in Thread
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {


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
    //System.out.println(mOperatorInterface.getDriveThrottle());
    //System.out.println(mOperatorInterface.getDriveTurn());
    //manual drive
    mDrive.setDrive(driveThrottle, driveTurn, false);
  }


}
