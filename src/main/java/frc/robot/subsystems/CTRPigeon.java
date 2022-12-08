package frc.robot.subsystems;

import frc.robot.util.loops.ILooper;
import com.ctre.phoenix.sensors.Pigeon2;

/*
All Subsystem classes must extend the subsystem abstract class
This will allow us to call common methods from the subsystem management
the constructor will automatically add to the subsystem manager
*/
public class CTRPigeon {
  private static CTRPigeon mInstance = null;

  Pigeon2 pigeon = new Pigeon2(0, "rio");

  public static synchronized CTRPigeon getInstance(){
    if (mInstance == null) {
      mInstance = new CTRPigeon();
      }
    return mInstance;
  }

  public void calibratePigeon(){
  
  }

  public double getPigeonYaw(){
    return pigeon.getYaw();
  }

  public double getPigeonRoll(){
    return pigeon.getRoll();
  }

  public double getPigeonPitch(){
    return pigeon.getPitch();
  }
}
