// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj.GenericHID;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {  

  // Controllers Reference
  private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();



  // Subsystem Manager
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  
  // Subsystems
  private final Drive mDrive = Drive.getInstance();

  private final Shooter mShooter = Shooter.getInstance();


  //Power dist. panel
  //private final PowerDistributionPanel m_pdp = new PowerDistributionPanel(25);

  // Mode
  public enum RobotMode {
    CargoScorer,
    Climber
  };
  public RobotMode mCurrentMode = RobotMode.CargoScorer;

  // Get Robot Mode Name to String
  public String modeToString(RobotMode s){
    switch(s){
      case CargoScorer:
        return "CargoScorer";
      case Climber:
        return "Climber";
      default:
        return "";
    }
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // populate autonomous list
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

    mSubsystemManager.updateSmartdashboards();
  }


  /** Called at the Start of Autonomous **/
  @Override
  public void autonomousInit() {
    mOperatorInterface.setRumble(false);

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

  private void RobotLoop(){
    DriveLoop();
    mShooter.setPower(mOperatorInterface.getShooterPower());
  }
  


  private void DriveLoop(){
    double driveThrottle = mOperatorInterface.getDriveThrottle();
    double driveTurn = mOperatorInterface.getDriveTurn();
    
    mDrive.setDrive(driveThrottle, driveTurn, false);
  }

}
