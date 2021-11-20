package frc.robot.OI;

import frc.robot.Constants;
import frc.robot.OI.NykoController.Axis;
import frc.robot.OI.NykoController.DPad;
import frc.robot.OI.XboxController.Button;
import frc.robot.OI.XboxController.Side;
import frc.robot.Robot;
import frc.robot.Constants.Controllers.Operator;
import frc.robot.util.LatchedBoolean;

public class OperatorInterface {
    private static OperatorInterface mInstance;

    // Instances of the Driver and Operator Controller
    private  XboxController DriverController;
    private  NykoController OperatorController;
    private final JoystickController joysticks;
    public static synchronized OperatorInterface getInstance() {
        if (mInstance == null) {
            mInstance = new OperatorInterface();
        }
        return mInstance;
    }
    
    private OperatorInterface() {
        /*DriverController = new XboxController(Constants.Controllers.Driver.port);
        OperatorController = new NykoController(Constants.Controllers.Operator.port);
        */
        joysticks = JoystickController.getInstance();
    }

    public double getDriveThrottle() {
        return joysticks.getYJoyStick_L();
      }
    
    public double getDriveTurn() {
        return joysticks.getXJoyStick_R();
    }
    public boolean isGoButtonPressed() {
        return joysticks.isButton4Pressed_L();
    }
    public boolean isBackButtonPressed() {
        return joysticks.isButton5Pressed_L();
    }
    public int shoulderExtension_value() {
        return joysticks.getYJoyStick_L();
    }
    public int armExtension_value() {
        return joysticks.getYJoyStick_R();
    }
}
