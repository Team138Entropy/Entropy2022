package frc.robot.OI;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


public class JoystickController {
    private static JoystickController mInstance;
    // Create the joystick and the 8 buttons on it

    public static synchronized JoystickController getInstance() {
        if (mInstance == null) {
            mInstance = new JoystickController();
      }
      return mInstance;
  }

   Joystick leftJoy = new Joystick(0);
   JoystickButton button1_L = new JoystickButton(leftJoy, 1),
    button2_L = new JoystickButton(leftJoy, 2),
    button3_L = new JoystickButton(leftJoy, 3),
    button4_L = new JoystickButton(leftJoy, 4),
    button5_L = new JoystickButton(leftJoy, 5),
    button6_L = new JoystickButton(leftJoy, 6),
    button7_L = new JoystickButton(leftJoy, 7),
    button8_L = new JoystickButton(leftJoy, 8),
    button9_L = new JoystickButton(leftJoy, 9),
    button10_L = new JoystickButton(leftJoy, 10),
    button11_L = new JoystickButton(leftJoy, 11);
   public double getZJoyStick_L() {
      return leftJoy.getZ(); 
      
   }
   public double getXJoyStick_L() {
      return leftJoy.getX();
   } 
   public double getYJoyStick_L() {
      return leftJoy.getY();
   }
   Joystick RightJoy = new Joystick(1);
   JoystickButton button1_R = new JoystickButton(RightJoy, 1),
    button2_R = new JoystickButton(RightJoy, 2),
    button3_R = new JoystickButton(RightJoy, 3),
    button4_R = new JoystickButton(RightJoy, 4),
    button5_R = new JoystickButton(RightJoy, 5),
    button6_R = new JoystickButton(RightJoy, 6),
    button7_R = new JoystickButton(RightJoy, 7),
    button8_R = new JoystickButton(RightJoy, 8),
    button9_R = new JoystickButton(RightJoy, 9),
    button10_R = new JoystickButton(RightJoy, 10),
    button11_R = new JoystickButton(RightJoy, 11);
   public double getZJoyStick_R() {
      return RightJoy.getZ();
   }
   public double getXJoyStick_R() {
      return RightJoy.getX();
   } 
   public double getYJoyStick_R() {
      return RightJoy.getY();
   }
   /*
   public boolean isButton1Pressed() {
      if (button1.get()){
         return true;
      }
      return false;
   }
   public boolean isButton2Pressed() {
      if (button2.get()){
         return true;
      }
      return false;
   }
   public boolean isButton3Pressed() {
      if (button3.get()){
         return true;
      }
      return false;
   }
   public boolean isButton4Pressed() {
      if (button4.get()){
         return true;
      }
      return false;
   }
   public boolean isButton5Pressed() {
      if (button5.get()){
         return true;
      }
      return false;
   }
   public boolean isButton6Pressed() {
      if (button6.get()){
         return true;
      }
      return false;
   }
   public boolean isButton7Pressed() {
      if (button7.get()){
         return true;
      }
      return false;
   }
   public boolean isButton8Pressed() {
      if (button8.get()){
         return true;
      }
      return false;
   }
   public boolean isButton9Pressed() {
      if (button9.get()){
         return true;
      }
      return false;
   }
   public boolean isButton10Pressed() {
      if (button10.get()){
         return true;
      }
      return false;
   }
   public boolean isButton11Pressed() {
      if (button11.get()){
         return true;
      }
      return false;
   }
   */
}