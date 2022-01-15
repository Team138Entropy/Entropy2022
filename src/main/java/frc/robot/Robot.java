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
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.DoNothingMode;
import frc.robot.auto.modes.TestDriveMode;



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
  //private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  
  // Subsystems
  private final Drive mDrive = Drive.getInstance();
  private final Arm mArm = Arm.getInstance();
  // Autonomous Execution Thread
  private AutoModeExecutor mAutoModeExecutor = null;

  // Autonomous Modes
  private SendableChooser<AutoModeBase> mAutoModes;

  private double shoulderRotatePoint = -60;

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
    mAutoModes.addOption("Nothing", new DoNothingMode());
    mAutoModes.addOption("Test Drive", new TestDriveMode());
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
    // set auto mode

    // Get Selected AutoMode
    AutoModeBase selectedMode = mAutoModes.getSelected();
    if(selectedMode == null){
      System.out.println("Selected Auto Mode is Null");
    }


    TestDriveMode tdm = new TestDriveMode();
    mAutoModeExecutor.setAutoMode(tdm);

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

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // Set Mode for the Auto Mode to use in Thread

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {


  }

  boolean isJog = false;

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // if (mOperatorInterface.getButton1()) {
    //   System.out.println("Button 1");
    //   shoulderRotatePoint = -50;
    // } else if (mOperatorInterface.getButton2()) {
    //   System.out.println("Button 2");
    //   shoulderRotatePoint = 3;
    // } else if (mOperatorInterface.getButton3()) {
    //   System.out.println("Button 3");
    //   shoulderRotatePoint = 63;
    // } else if (mOperatorInterface.getButton4()) {
    //   System.out.println("Button 4");
    //   shoulderRotatePoint = 177;
    // }

    if (mOperatorInterface.getArmExtend()) {
      mArm.extend(.8);
    } else if (mOperatorInterface.getArmRetract()) {
      mArm.extend(-.8);
    }

    double magnitude = Math.pow(Math.pow(mOperatorInterface.getOperatorThrottle(), 2) + Math.pow(mOperatorInterface.getOperatorTurn(), 2), .5);
    double direction = Math.atan(mOperatorInterface.getOperatorThrottle() / mOperatorInterface.getOperatorTurn());
    direction *= 1 / Constants.Misc.degreeToRadian;

    if (mOperatorInterface.getOperatorTurn() < 0) {
      direction += 180;
    }

    int[] directions = {-50, 0, 60, 90, 120, 210};
    
    int error = 360;
    for (int i : directions) {
      if (magnitude < .5) break;
      if (Math.abs(i - (int) direction) < error) {
        shoulderRotatePoint = i - 3;
        error = Math.abs(i - (int) direction);
      }
    }
    
    // Make the arm move
    mArm.rotateShoulderPosition(shoulderRotatePoint);
    
    SmartDashboard.putNumber("throttleAngle", direction);
    SmartDashboard.putNumber("velocity", mArm.getShoulderVelocity());
    SmartDashboard.putNumber("throttle", mOperatorInterface.getOperatorThrottle());
    SmartDashboard.putNumber("shoulderTarget", shoulderRotatePoint);
    SmartDashboard.putNumber("shoulderPosition", mArm.getShoulderPosition());
    SmartDashboard.putNumber("shoulderOutput", mArm.getShoulderOutput());
    SmartDashboard.putNumber("feedForward", mArm.getGravityFeedForward());
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
