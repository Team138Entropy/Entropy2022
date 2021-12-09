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
    boolean XboxInUse = false;
    // Instances of the Driver and Operator Controller
    private XboxController DriverController;
    private NykoController OperatorController;
    private JoystickController joysticks;
    public static synchronized OperatorInterface getInstance() {
        if (mInstance == null) {
            mInstance = new OperatorInterface();
        }
        return mInstance;
    }
    
    private OperatorInterface() {
        DriverController = new XboxController(Constants.Controllers.Driver.port);
        OperatorController = new NykoController(Constants.Controllers.Operator.port);
        joysticks = JoystickController.getInstance();
    }
    public void setXboxInUse(boolean value) { 
    XboxInUse = value;
    }
    public boolean getXboxInUse() {
        return XboxInUse;
    }
    public double getDriveThrottle() {
        if (XboxInUse = true){
             return DriverController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
        } 
        else 
            {return joysticks.getYJoyStick_L();}
      }
    
    public double getDriveTurn() {
        if (XboxInUse = true) {
             return DriverController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
        }
        else
            {return joysticks.getXJoyStick_L();}
    }

}
