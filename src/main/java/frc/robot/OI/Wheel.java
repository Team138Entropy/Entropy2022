package frc.robot.OI;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Wheel {

    private static Wheel mInstance;

    public static synchronized Wheel getInstance() {
        if (mInstance == null) {
            mInstance = new Wheel();
      }
      return mInstance;
  }

   Joystick wheel = new Joystick(5);
   JoystickButton button1_W = new JoystickButton(wheel, 1),
    button2_W = new JoystickButton(wheel, 2),
    button3_W = new JoystickButton(wheel, 3),
    button4_W = new JoystickButton(wheel, 4),
    button5_W = new JoystickButton(wheel, 5),
    button6_W = new JoystickButton(wheel, 6),
    button7_W = new JoystickButton(wheel, 7),
    button8_W = new JoystickButton(wheel, 8),
    button9_W = new JoystickButton(wheel, 9),
    button10_W = new JoystickButton(wheel, 10),
    button11_W = new JoystickButton(wheel, 11),
    button12_W = new JoystickButton(wheel, 12);

   public double getXWheel() {
      return wheel.getX();
   } 
   public double getYWheel() {
      return wheel.getY();
   }

   public boolean isButton1Pressed_W() {
      if (button1_W.get()){
         return true;
      }
      return false;
   }

   public boolean isButton2Pressed_W() {
      if (button2_W.get()){
         return true;
      }
      return false;
   }

      public boolean isButton3Pressed_W() {
      if (button3_W.get()){
         return true;
      }
      return false;
   }

      public boolean isButton4Pressed_W() {
      if (button4_W.get()){
         return true;
      }
      return false;
   }

      public boolean isButton5Pressed_W() {
      if (button5_W.get()){
         return true;
      }
      return false;
   }

      public boolean isButton6Pressed_W() {
      if (button6_W.get()){
         return true;
      }
      return false;
   }

   public boolean isButton7Pressed_W() {
      if (button7_W.get()){
         return true;
      }
      return false;
   }

   public boolean isButton8Pressed_W() {
      if (button8_W.get()){
         return true;
      }
      return false;
   }

      public boolean isButton9Pressed_W() {
      if (button9_W.get()){
         return true;
      }
      return false;
   }

      public boolean isButton10Pressed_W() {
      if (button10_W.get()){
         return true;
      }
      return false;
   }

      public boolean isButton11Pressed_W() {
      if (button11_W.get()){
         return true;
      }
      return false;
   }

      public boolean isButton12Pressed_W() {
      if (button12_W.get()){
         return true;
      }
      return false;
   }
}