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

  private double shoulderTarget = -60;

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

  private boolean isAutoExtending = false;

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    
    // The following two arm control modes don't both work at once
    if (mOperatorInterface.getArmExtend()) {
      mArm.extend(.8);
      System.out.println("Extending");
      isAutoExtending = true;
    } else if (mOperatorInterface.getArmRetract()) {
      mArm.extend(-.8);
      isAutoExtending = true;
    }
    
    if (mOperatorInterface.getArmExtendManual()) {
      mArm.extend(.5);
      isAutoExtending = false;
    } else if (mOperatorInterface.getArmRetractManual()) {
      mArm.extend(-.5);
      isAutoExtending = false;
    } else if (!isAutoExtending) {
      mArm.stopForearm();
    }

    /* Joystick control of the arm */
    int[] directions = {-50, 0, 60, 90, 120, 210}; // We are using a fixed list of points to navigate to
    
    // Convert the joystick x and y positions into a 2D vector. Magnitude = sqrt(x^2+y^2).
    // Angle (in radians) = arctangent(y / x)
    double magnitude = Math.sqrt(Math.pow(mOperatorInterface.getOperatorStickY(), 2) + Math.pow(mOperatorInterface.getOperatorStickX(), 2));
    double angle = Math.atan(mOperatorInterface.getOperatorStickY() / mOperatorInterface.getOperatorStickX());
    
    // Java arctangent method returns a number between -pi/2 to pi/2 (-90 to 90 degrees), so we have to convert to
    // degrees. We have to add 180 degrees if we want the arm to be able to rotate past 90
    angle *= -1 / Constants.Misc.degreeToRadian;
    if (mOperatorInterface.getOperatorStickX() > 0) angle += 180;

    int difference = 360; // The difference between each listed angle, and the actual joystick angle, used to find which listed angle is closest to the joystick
    for (int i : directions) {
      if (magnitude < .5) break; // If they aren't moving the joystick, don't move the arm
      if (Math.abs(i - (int) angle) < difference) {
        shoulderTarget = i + (i < 90 ? 3 : -3); // Offset the target by 3 degrees upwards to compensate for gravity/slop
        difference = Math.abs(i - (int) angle); // Set the new difference
      }
    }
    /* Joystick control of the arm */
    
    // Make the arm move
    mArm.rotateShoulderPosition(shoulderTarget);
    
    SmartDashboard.putBoolean("dpadUp", mOperatorInterface.getArmExtendManual());
    SmartDashboard.putNumber("dpad", mOperatorInterface.getDPadValue());
    SmartDashboard.putNumber("forearmOutput", mArm.getForearmOutput());
    SmartDashboard.putNumber("shoulderTarget", shoulderTarget);
    SmartDashboard.putNumber("shoulderVelocity", mArm.getShoulderVelocity());
    SmartDashboard.putNumber("shoulderPosition", mArm.getShoulderPosition());
    SmartDashboard.putNumber("shoulderOutput", mArm.getShoulderOutput());
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
