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
    int controllerInUse = 0;
    // Instances of the Driver and Operator Controller
    private XboxController DriverController;
    private NykoController OperatorController;
    private JoystickController joysticks;
    private Wheel wheel;
    private uselessController controllerThing;
    public static synchronized OperatorInterface getInstance() {
        if (mInstance == null) {
            mInstance = new OperatorInterface();
        }
        return mInstance;
    }
    
    private OperatorInterface() {
        DriverController = new XboxController(3);
        OperatorController = new NykoController(Constants.Controllers.Operator.port);
        joysticks = JoystickController.getInstance();
        wheel = Wheel.getInstance();
        controllerThing = uselessController.getInstance();
    }
    public void setControllerInUse(int value) { 
    controllerInUse = value;
    System.out.println(controllerInUse);
    }
    public int getControllerInUse() {
        return controllerInUse;
    }
    public double getDriveThrottle() {
        if (controllerInUse == 0){
             return DriverController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
        } 
        if (controllerInUse == 1){
            return joysticks.getYJoyStick_L();
        }
        if (controllerInUse == 2){
            return wheel.getYWheel();
        }
        if (controllerInUse == 3){
            return controllerThing.getY_thing();
        }
        return 0;
      }
    
    public double getDriveTurn() {
        if (controllerInUse == 0) {  
             return DriverController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
        }
        if (controllerInUse == 1) {
             return joysticks.getXJoyStick_R();
        }
        if (controllerInUse == 2){
            return wheel.getXWheel();
        }
        if (controllerInUse == 3){
            return controllerThing.getX_thing();
        }
        return 0;
    }

}
