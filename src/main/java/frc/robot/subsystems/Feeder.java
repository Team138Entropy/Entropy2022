package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.Enums.JogDirection;
import frc.robot.util.drivers.EntropyTalonFX;

public class Feeder extends Subsystem {
  private static Feeder mInstance;

  public static synchronized Feeder getInstance() {
    if (mInstance == null) {
      mInstance = new Feeder();
    }
    return mInstance;
  }

  // Talons
  private final EntropyTalonFX mShooterInputTalon;
  private final EntropyTalonFX mFeeder1Talon;
  private final EntropyTalonFX mFeeder2Talon;

  private Feeder() {
    // Init Talons
    mShooterInputTalon = new EntropyTalonFX(Constants.Talons.Feeder.shooterInput);
    mFeeder1Talon = new EntropyTalonFX(Constants.Talons.Feeder.feeder1);
    mFeeder2Talon = new EntropyTalonFX(Constants.Talons.Feeder.feeder2);
    mShooterInputTalon.setInverted(true);
    mFeeder1Talon.setInverted(true);
    mFeeder2Talon.setInverted(true);
  }

  // Set Shooter Jogging
  public synchronized void jogShooterInput(JogDirection direction)
  {
    double speed = 0;
    switch(direction)
    {
      case FORWARD:
        speed = Constants.Feeder.inputWheelTestSpeed_Forward.get();
      break;
      case REVERSE:
        speed = Constants.Feeder.inputWheelTestSpeed_Reverse.get();
      break;
      case STOP:
        speed = 0;
      break;
    }
    mShooterInputTalon.set(ControlMode.PercentOutput, speed);
  }

  public synchronized void jogFeeder(JogDirection direction)
  {
    double feeder1Speed = 0;
    double feeder2Speed = 0;
    switch(direction)
    {
      case FORWARD:
        feeder1Speed = Constants.Feeder.feederTestSpeed_Forward.get();
        feeder2Speed = Constants.Feeder.feederTestSpeed_Forward.get();
      break;
      case REVERSE:
        feeder1Speed = Constants.Feeder.feederTestSpeed_Reverse.get();
        feeder2Speed = Constants.Feeder.feederTestSpeed_Reverse.get();
      break;
      case STOP:
        feeder1Speed = 0;
        feeder2Speed = 0;
      break;
    }
    mFeeder1Talon.set(ControlMode.PercentOutput, feeder1Speed);
    mFeeder2Talon.set(ControlMode.PercentOutput, feeder2Speed);
  }




  public void zeroSensors() {

  }

  public void checkSubsystem() {


  }

  public void updateSmartDashBoard() {
    mShooterInputTalon.updateSmartdashboard();
    mFeeder1Talon.updateSmartdashboard();
    mFeeder2Talon.updateSmartdashboard();
  }
}