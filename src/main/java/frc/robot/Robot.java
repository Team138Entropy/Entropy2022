// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.RamseteController;
import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.*;
import frc.robot.auto.AutoModeExecutor;



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

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

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

  }


  /** Called at the Start of Autonomous **/
  @Override
  public void autonomousInit() {
    // Start Autonomous Thread
    // This thread will run until disabled
    mAutoModeExecutor.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // Disable Auto Thread (if running)
    if (mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
    }

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

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // Set Mode for the Auto Mode to use in Thread
    //mAutoModeExecutor.setAutoMode(new_auto_mode);

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

    //manual drive
    mDrive.setDrive(driveThrottle, driveTurn, false);
  }


}
