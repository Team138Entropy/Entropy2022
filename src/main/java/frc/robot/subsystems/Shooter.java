package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.Enums.JogDirection;
import frc.robot.util.drivers.EntropyTalonFX;

public class Shooter extends Subsystem {
  private static Shooter mInstance;

  private final EntropyTalonFX mLeftTalon;
  private final EntropyTalonFX mRightTalon;

  public static synchronized Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  private Shooter() {
    mLeftTalon = new EntropyTalonFX(Constants.Talons.Shooter.leftMaster);
    mRightTalon = new EntropyTalonFX(Constants.Talons.Shooter.rightMaster);
    mRightTalon.setInverted(true);
  }

  // Set Shooter Jogging
  public synchronized void jogShooterInput(JogDirection direction)
  {
    double speed = 0;
    switch(direction)
    {
     case FORWARD:
       speed = Constants.Shooter.shooterTestSpeed_Forward.get();
     break;
     case REVERSE:
       speed = Constants.Shooter.shooterTestSpeed_Reverse.get();
     break;
     case STOP:
        speed = 0;
     break;
    }
    mLeftTalon.set(ControlMode.PercentOutput, speed);
    mRightTalon.set(ControlMode.PercentOutput, speed);
   }

  public void zeroSensors() {

  }

  public void checkSubsystem() {


  }

  public void updateSmartDashBoard() {
    mLeftTalon.updateSmartdashboard();
    mRightTalon.updateSmartdashboard();
  }
}