// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.*;

//remove
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.FileWriter;
import frc.robot.util.DriveSignal;

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


  boolean firstLoop = true;
  long startTimeMilliseconds = System.currentTimeMillis();

  FileWriter csvWriter;
  /** Called at the Start of Autonomous **/
  @Override
  public void autonomousInit() {
    try{
      csvWriter = new FileWriter("WaltersDrive0to1.csv");
      csvWriter.append("Milliseconds,");
      csvWriter.append("Left Output (),");
      csvWriter.append("Right Output (),");
      csvWriter.append("\n");
      csvWriter.flush();
    }catch(java.io.IOException e){
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if(firstLoop){
      startTimeMilliseconds = System.currentTimeMillis();
      firstLoop = false;
    }

    //System.out.println("Drive!");
    //test period
    //mDrive.setDrive(1, 0, false);
    long timeMilliseconds = System.currentTimeMillis() - startTimeMilliseconds;
    DriveSignal d = mDrive.getCheesyBrianDrive(1, 0, false);

    try{
      csvWriter.append(Long.toString(timeMilliseconds));
      csvWriter.append(",");


      // Write Output
      csvWriter.append(Double.toString(d.getLeft()));
      csvWriter.append(",");
      csvWriter.append(Double.toString(d.getRight()));
      csvWriter.append(",");


      csvWriter.append("\n");
      csvWriter.flush();
    }catch(java.io.IOException e){

    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    teleopRobotLoop();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {


  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {


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
