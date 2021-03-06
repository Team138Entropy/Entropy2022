package frc.robot.subsystems;

import frc.robot.util.loops.ILooper;

/*
All Subsystem classes must extend the subsystem abstract class
This will allow us to call common methods from the subsystem management
the constructor will automatically add to the subsystem manager
*/
public abstract class Subsystem {

  public Subsystem() {
    SubsystemManager.getInstance().registerSubsystem(this);
  }

  public abstract void zeroSensors();

  public abstract void checkSubsystem();
  
  public abstract void updateSmartDashBoard();

  // Optional Design Pattern. which gets called from the looper
  // part of this design is to avoid over utiuilization of the can
  // these methods are not abstract so they don't need to implimented

  public void readPeriodicInputs() {}

  public void writePeriodicOutputs() {}

  public void registerEnabledLoops(ILooper mEnabledLooper) {}

  // called from subsystem manager
  public void stopSubsytem() {}
}
