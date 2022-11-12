package frc.robot.subsystems;

public class ExampleSubsystem extends Subsystem {
  private static ExampleSubsystem mInstance;

  public static synchronized ExampleSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ExampleSubsystem();
    }
    return mInstance;
  }

  private ExampleSubsystem() {

  }

  public void zeroSensors() {

  }

  public void checkSubsystem() {


  }

  public void updateSmartDashBoard() {

  }
}