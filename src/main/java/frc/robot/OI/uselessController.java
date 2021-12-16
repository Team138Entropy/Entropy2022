package frc.robot.OI;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class uselessController {
    private static uselessController mInstance;

    public static synchronized uselessController getInstance() {
        if (mInstance == null) {
            mInstance = new uselessController();
      }
      return mInstance;
  }

  Joystick controllerThing = new Joystick(4);

 public double getX_thing() {
      return controllerThing.getX();
   } 
public double getY_thing() {
      return controllerThing.getY();
   }
}